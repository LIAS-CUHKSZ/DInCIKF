# DInCIKF
Distributed Invariant Kalman Filter based on Covariance Intersection for multi robots cooreperatively state estimation.

## Important Files

**main.mlx**: the implement of DInCIKF. I also provide the comparison between three other methods. Choose which methods you want to compare and make it 1.

```matlab
%      IndepOdom|InEKF|D-InEKF-fast-CI|EKF|  
Method= [   0,       1,       1,        1];
```

**beautiful_scene.mlx**: Draw the pose frames of the test scene and the objects in the environment. Also it will generate a video of the estimation result. This script must be run after running the main.mlx with  SAVEVIDEO set to be 1;




## Generate the datasets

Run  [Scenes/generate_scene.mlx](). You can modify the number of agents, trajectory, noise scale, graph structure and so on. The dataset will be automatically saved in the [dataset]() file.

We also put the test scenes in our paper in the [dataset]() file.

Before run the main code, change your dataset you want to test in [main.mlx](). For example,

```matlab
datasetname='data18';   
datapath=['dataset\',datasetname,'.mat'];
```

 The test results will be saved in file TestResults.  

## Related Paper

Arxiv version:

https://arxiv.org/abs/2409.07933

```
@article{li2024covariance,
  title={Covariance Intersection-based Invariant Kalman Filtering (DInCIKF) for Distributed Pose Estimation},
  author={Li, Haoying and Li, Xinghan and Huang, Shuaiting and Wu, Junfeng and others},
  journal={arXiv preprint arXiv:2409.07933},
  year={2024}
}
```

*This paper is accepted by 2024 Conference on Decision and Control.*
