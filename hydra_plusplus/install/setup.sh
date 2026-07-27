#!/usr/bin/env bash

pkg_dir="$(dirname $(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd))"
shell="$(basename ${SHELL})"

install_crisp=true

while :; do
    case $1 in
        --no-crisp)
            install_crisp=false
            shift
            ;;
        -h|-?|--help)
            echo "Usage: setup.sh [--no-crisp]"
            exit
            ;;
        *)
            break
    esac
done

if [ "$install_crisp" = true ]; then
    echo "Package: ${pkg_dir}"
    python3 -m virtualenv --clear --system-site-packages "${pkg_dir}/environments/crisp"
    source "${pkg_dir}/environments/crisp/bin/activate"
    pip install -r "${pkg_dir}/install/crisp_requirements.txt"
    pip install -e "${pkg_dir}/crisp"
    deactivate
fi

user_config="$HOME/.bashrc"
case $shell in
    bash)
        echo "Exporting variables to .bashrc"
        user_config="$HOME/.bashrc"
        ;;
    zsh)
        echo "Exporting variables to .zshrc"
        user_config="$HOME/.zshrc"
        ;;
    *)
        echo "Unknown shell '$shell'!"
        exit
esac

echo "export PATH="'$PATH:'"${pkg_dir}/bin" >> "$user_config"
echo "export CRISP_ENV=${pkg_dir}/environments/crisp" >> "$user_config"
echo "export CRISP_MODEL_DIR=${pkg_dir}/models/" >> "$user_config"
