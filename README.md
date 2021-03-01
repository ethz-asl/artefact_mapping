# abb_odn
Obj-det kABBobi

## Install
* Install [`git lfs`](https://packagecloud.io/github/git-lfs/install)
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
```
* Reproduce all steps of the [maplab install](https://maplab.asl.ethz.ch/docs/develop/pages/installation/A_Installation-Ubuntu.html) instructions up to the building of maplab itself.
* `git clone --recursive git@github.com:ethz-asl/abb_odn.git`
* `catkin build abb_odn`
* `rosrun abb_odn abb_odn`
