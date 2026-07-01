#!/usr/bin/env python3
"""Node that runs crisp on received masks and RGBD data."""

import pathlib

import geometry_msgs.msg
import numpy as np
import torch
import torch.onnx
import torchvision.transforms.v2 as tvt
from crisp.config.config import BaseExpSettings
from crisp.models.certifier import FrameCertifier
from crisp.models.joint import JointShapePoseNetwork
from crisp.models.pipeline import Pipeline
from crisp.models.rmcc import ReprojectionMaskConsistencyChecker
from crisp.utils.constants import IMAGENET_DEFAULT_MEAN, IMAGENET_DEFAULT_STD
from crisp.utils.sdf import convert_sdf_samples_to_mesh, create_sdf_samples_generic
from scipy.spatial.transform import Rotation as Rot


def export_model(model):
    fake_coords = torch.rand((1, 1, 1, 1), dtype=torch.float32)
    shape_code = torch.rand((1, 256), dtype=torch.float32)
    torch.onnx.export(
        model.recons_net,
        (shape_code, fake_coords),
        "model.onnx",
        input_names=["shape_code", "coords"],
    )


def setup_pipeline(
    checkpoint, device, width, height, log_path=None, debug_settings=None
):
    opt = BaseExpSettings(dataset_dir="./")  # note: dataset_dir not actually used
    opt.image_size = (height, width)
    opt.checkpoint_path = pathlib.Path(checkpoint).expanduser().absolute()
    opt.use_corrector = False

    # joint nocs & recons model
    model = JointShapePoseNetwork(
        input_dim=3,
        recons_num_layers=5,
        recons_hidden_dim=256,
        recons_modulate_last_layer=True,
        recons_modulate_type="film",
        backbone_model=opt.backbone_model_name,
        local_backbone_model_path=None,
        freeze_pretrained_weights=True,
        backbone_input_res=opt.backbone_input_res,
        nonlinearity=opt.recons_nonlinearity,
        normalization_type=opt.recons_normalization_type,
        nocs_network_type=opt.nocs_network_type,
        nocs_channels=opt.nocs_channels,
        lateral_layers_type=opt.nocs_lateral_layers_type,
        normalize_shape_code=opt.recons_shape_code_normalization,
        recons_shape_code_norm_scale=opt.recons_shape_code_norm_scale,
    )

    state = torch.load(opt.checkpoint_path)
    model.load_state_dict(state["model"])
    model.eval()
    model.backbone.eval()
    model.cuda()

    frame_certifier = FrameCertifier(
        model=model,
        depths_clamp_thres=opt.cert_depths_clamp_thres,
        depths_quantile=opt.cert_depths_quantile,
        depths_eps=opt.cert_depths_eps,
        degen_min_eig_thres=opt.cert_degen_min_eig_thres,
        use_degen_cert=opt.use_degen_cert,
        shape_code_library=None,
    )

    rmcc_verifier = ReprojectionMaskConsistencyChecker(
        model=model,
        cube_samples=32,
        cube_scale=3.0,
        projection_radius=1,
        thr_num_pixels_in_mask=0,
        thr_iou=0.0,
        # NOTE(hlim): Using `thr_precision` showed the best performance
        thr_precision=0.1,
        # thr_precision=0.0, # NOTE (multy) set to 0.0 to disable and debug
        thr_coverage=0.0,
        vis_output=False,
        log_path=log_path,
    )

    pipeline = Pipeline(
        model=model,
        corrector=None,
        frame_certifier=frame_certifier,
        frame_corrector_mode=opt.frame_corrector_mode,
        pgo_solver=None,
        multi_frame_shape_code_corrector=None,
        multi_frame_geometric_shape_corrector=None,
        device=device,
        rmcc_verifier=rmcc_verifier,
        input_H=opt.image_size[0],
        input_W=opt.image_size[1],
        nr_downsample_before_corrector=opt.pipeline_nr_downsample_before_corrector,
        sdf_input_loss_multiplier=opt.corrector_loss_multiplier,
        registration_inlier_thres=opt.registration_inlier_thres,
        output_intermediate_vars=True,
        output_precrt_results=opt.pipeline_output_precrt_results,
        ssl_batch_size=opt.ssl_batch_size,
        ssl_nocs_clamp_quantile=opt.ssl_nocs_clamp_quantile,
        ssl_augmentation=False,
        ssl_augmentation_type=opt.ssl_augmentation_type,
        ssl_augmentation_gaussian_perturb_std=opt.ssl_augmentation_gaussian_perturb_std,
        no_grad_model_forward=True,
        normalized_recons=True,
        output_degen_condition_number=False,
        normalize_input_image=False,
        shape_code_library=None,
        debug_settings=debug_settings,
    )

    return pipeline


def get_image_transform():
    return tvt.Compose(
        [
            tvt.ToImage(),
            tvt.ToDtype(torch.float32, scale=True),
            tvt.Normalize(mean=IMAGENET_DEFAULT_MEAN, std=IMAGENET_DEFAULT_STD),
        ]
    )


def decompose_mesh_request(request, device):
    vec = torch.tensor(request.shape_code).unsqueeze(0).float().to(device)
    return vec, request.scale


def reconstruct_mesh(model, shape_code, s_nocs, cube_samples, cube_scale):
    def callback(coords):
        return model.recons_net.forward(shape_code=shape_code, coords=coords)

    (sdf_grid, voxel_size, voxel_grid_origin) = create_sdf_samples_generic(
        model_fn=callback,
        N=cube_samples,
        max_batch=64**3,
        cube_center=np.array([0, 0, 0]),
        cube_scale=cube_scale,
    )

    mesh = convert_sdf_samples_to_mesh(
        sdf_grid=sdf_grid,
        voxel_grid_origin=voxel_grid_origin,
        voxel_size=voxel_size,
    )

    mesh.vertices *= s_nocs
    return mesh


def pose_from_payload(payload):
    pose = geometry_msgs.msg.Pose()
    t = np.squeeze(payload["cam_t_nocs"][0, ...].cpu().numpy())
    pose.position.x = float(t[0])
    pose.position.y = float(t[1])
    pose.position.z = float(t[2])
    R = np.squeeze(payload["cam_R_nocs"][0, :3, :3].cpu().numpy())
    q = Rot.from_matrix(R).as_quat()
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose
