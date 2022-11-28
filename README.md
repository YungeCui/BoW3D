# BoW3D
BoW3D is developed for the real-time loop closing in 3D LiDAR SLAM. It builds the bag of words for the **LinK3D Feature** **[PDF](https://arxiv.org/pdf/2206.05927.pdf)**, which is an efficient and robust 3D LiDAR point cloud feature. BoW3D is able to detect loops and correct the full 6-DoF loop relative pose for the subsequent pose graph optimization in real time. It also can be used to relocalization the 3D LiDAR in real time. We provide an example to run BoW3D on the [KITTI dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php). For more details about BoW3D, please refer to our paper "BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM" in IEEE Robotics and Automation Letters (RA-L) **[PDF](https://arxiv.org/pdf/2208.07473.pdf)**.

<pre><code class="language-css">
@ARTICLE{9944848,
  author={Cui, Yunge and Chen, Xieyuanli and Zhang, Yinlong and Dong, Jiahua and Wu, Qingxiao and Zhu, Feng},
  journal={IEEE Robotics and Automation Letters}, 
  title={BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM}, 
  year={2022},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2022.3221336}}
</code></pre>



pre[class*="language-"] {
  position: relative;
  margin: 5px 0 ;
  padding: 1.75rem 0 1.75rem 1rem;

  /* more stuff */
}

pre[class*="language-"] button{
  position: absolute;
  top: 5px;
  right: 5px;

  /* more stuff */
}
