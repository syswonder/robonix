# make sure the script runs in the root of robonix
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

export PYTHONPATH="$PWD:$PYTHONPATH"
bash init.sh

# ensure all ros2 processes are stopped
bash stop.sh

cd robonix
python3 manager/boot.py --config ../config/include/ranger_test.yml
