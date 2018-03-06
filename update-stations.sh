# Can specifiy a particular branch to propagate to machines as the first argument.
# Defaults to master if no argument is supplied


for i in serl@station1 serl@station3 serl@station4 station@station5 serl@azeroth
do
    echo $i
    ssh $i << 'ENDSSH'
cd ~/sarwai-experiment
git stash apply "Changes auto-stashed by update-stations.sh"
git checkout darknet-detection
git pull origin darknet-detection
source devel/setup.bash
catkin_make
ENDSSH
done
