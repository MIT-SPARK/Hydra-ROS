#!/usr/bin/env bash

pkg_dir="$(dirname "$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)")"
shell="$(basename "${SHELL}")"

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
    echo "Installing crisp in: ${pkg_dir}"
    python3 -m virtualenv --clear --system-site-packages "${pkg_dir}/env"
    "${pkg_dir}/env/bin/python3" -m pip install -r "${pkg_dir}/install/crisp_requirements.txt"
    "${pkg_dir}/env/bin/python3" -m pip install git+https://github.com/MIT-SPARK/CRISP
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
echo "export HYDRAPP_ENV=${pkg_dir}/env" >> "$user_config"
echo "export HYDRAPP_MODEL_DIR=${pkg_dir}/models/" >> "$user_config"
