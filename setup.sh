#!/bin/bash
# Recreate the full working setup under ~/SIH from this repository:
# downloads the third-party code at the exact versions we used, then applies our changes.
# Usage:  ./setup.sh            (creates ~/SIH; refuses if ~/SIH/bot_basics2 already exists)
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
T=$HOME/SIH
if [ -e "$T/bot_basics2" ]; then echo "ERROR: $T/bot_basics2 already exists - not overwriting."; exit 1; fi
mkdir -p $T/bot_basics2/src $T/forest_world $T/ML_Models/HRNet+OCR $T/ML_Models/MiDaS $T/launch

clone() {  # url  commit  destination
  git clone -q "$1" "$3" && git -C "$3" checkout -q "$2" && echo "  $(basename $3) @ $2"
}
echo "Fetching third-party code (pinned versions):"
S=$T/bot_basics2/src
clone https://github.com/husarion/rosbot_ros.git                      f9b9200 $S/rosbot_ros
clone https://github.com/husarion/husarion_components_description.git 5f783f8 $S/husarion_components_description
clone https://github.com/husarion/husarion_controllers                724f427 $S/husarion_controllers
clone https://github.com/husarion/husarion_gz_worlds                  4f1fdd1 $S/husarion_gz_worlds
clone https://github.com/husarion/husarion_asset_msgs.git             45e1d68 $S/husarion_asset_msgs
clone https://github.com/husarion/tf_namespace_bridge                 48ea8cd $S/tf_namespace_bridge
clone https://github.com/unitsSpaceLab/Forest3D.git                   5c3d331 $T/forest_world/Forest3D
git clone -q --filter=blob:none --sparse https://github.com/unmannedlab/RELLIS-3D.git $T/ML_Models/rellis_repo
git -C $T/ML_Models/rellis_repo sparse-checkout set benchmarks/HRNet-Semantic-Segmentation-HRNet-OCR
git -C $T/ML_Models/rellis_repo checkout -q c17a118 && echo "  rellis_repo @ c17a118 (HRNet code only)"

echo "Applying our changes:"
cp -r $REPO/overlay/rosbot_ros/.                      $S/rosbot_ros/
cp -r $REPO/overlay/husarion_components_description/. $S/husarion_components_description/
cp -r $REPO/overlay/Forest3D/.                        $T/forest_world/Forest3D/
cp $REPO/launch/* $T/launch/ && chmod +x $T/launch/*.sh
cp $REPO/ml/HRNet_OCR/*.yaml "$T/ML_Models/HRNet+OCR/"
cp $REPO/ml/MiDaS/midas_script.py $T/ML_Models/MiDaS/
echo "  done"

cat << MSG

Setup files are in place under $T. Next:
  1. Model weights: put best.pth in $T/ML_Models/HRNet+OCR/  (download link in README)
  2. Build:  cd $T/bot_basics2 && rosdep install --from-paths src -y --ignore-src && colcon build
  3. Nav2 settings:  python3 $T/launch/make_nav2_params.py
  4. Run:  $T/launch/start_sim.sh   then   $T/launch/start_pipeline.sh
MSG
