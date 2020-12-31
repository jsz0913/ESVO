一、文件组织
	主要esvo_time_surface和esvo_core两个功能包。
esvo_core中，有四个主要文件夹，分别为container、core、optimization、tools，还有mapping和tracking两个节点。
 

二、tools
2.1 TicToc.h
	1.TicToc类，用于计时。
		成员：std::chrono::time_point<std::chrono::system_clock>  start，end
		函数：构造函数tic，成员函数toc返回start和end经过秒数
2.2 sobel.h sobel.cpp
	1.sobel类，用于sobel边缘检测。
		成员：核大小         size_t kernel_size_;
  	          xy边缘检测矩阵static const Eigen::Matrix3d sobel_3x3_x, sobel_3x3_y;
  				        static const Eigen::Matrix<double,5,5> sobel_5x5_x, sobel_5x5_y;

		函数：convolve 将输入矩阵与检测矩阵对应相乘求和，得到卷积值
			  grad_y,grad_x分别调用convolve，计算x，y上卷积值
			  grad_xy 得到二维向量
2.3 cayley.h cayley.cpp
	1.提供两个函数，用于矩阵和向量的转换
		cayley2rot，rot2cayley
 

2.4 Visualization.h Visualization.cpp 用于事件的画图
包含container中的DepthMap
	1.枚举类型VisMapType，包含InvDepthMap,StdVarMap,CostMap,AgeMap
	2. Visualization类
		成员：static const float r[];static const float g[];static const float b[]; 用来染色
		函数：
plot_map，
输入SmartGrid<DepthPoint>的指针，枚举类型，range，阈值
输出大小相同的颜色图cv::mat		
遍历SmartGrid<DepthPoint>中_elements的所有深度点，阈值检测其有效性，调用DrawPoint，将对应深度/方差残差/年龄、range、坐标、cv::mat输入进去
DrawPoint，
利用给定range和值，将其划分到0-255，利用成员r\g\b得到颜色，在坐标周围画实心圆
plot_eventMap，2D图，事件坐标黑色。
plot_events，与plot_eventMap区别在于，输入量为存储二维矩阵的向量，同时判定是否在图像上
2.5 params_helper.h
	1.模板函数param，输入nodehandle和string，返回找到的参数值
2.6 utils.h
	包含container的SmartGrid.h，DepthPoint.h
	using Transformation = kindr::minimal::QuatTransformation;
	1.宏定义NUM_THREAD_TRACKING 1 NUM_THREAD_MAPPING 4
	2. inline static，EventBuffer_lower_bound，返回给定时间后事件队列的下界
	3. static inline void meanStdDev 计算矩阵块上的mean和sigma
4. static inline void normalizePatch 
5. static inline void _mkdir   递归的创建目录

三、container
3.1 ResidualItem.h ResidualItem.cpp
	1.结构体ResidualItem
		成员：Eigen::Vector3d p_      参考帧3D坐标
Eigen::Vector2d p_img_  像平面2D坐标
Eigen::VectorXd residual_ 
3.2 DepthPoint.h DepthPoint.cpp  
	1.类DepthPoint
		成员：size_t row_;size_t col_;
  			  Eigen::Vector2d x_;    像平面坐标
  			  //inverse depth parameters
  	  		  double invDepth_;
  			  double scaleSquared_;// squared scale
  	 		  double nu_;
  			  double variance_;
  		  	  double residual_;
  			  // count the number of fusion has been applied on a depth point
 		      size_t age_;

    //3D point (updated in reference frame before tracking)
    Eigen::Vector3d p_cam_;      相机坐标系3D
    Eigen::Matrix<double, 4, 4> T_world_cam_;
		函数：
			 点成员均提供了函数调用，还有普通函数和const 函数 const
			 //赋值函数和整体拷贝函数
			 void update_x(const Eigen::Vector2d &x);
			 void update_p_cam(const Eigen::Vector3d &p);
			 void updatePose(Eigen::Matrix<double, 4, 4> &T_world_cam);
			 void copy(const DepthPoint &copy)
			 //设定方差下限为1e-6
			 boundVariance()  
			 //论文中的student分布的点融合
			 void update_studentT(double invDepth, double scale2, double variance, double      nu)
			//通过逆深度、方差、age来检测点有效性
			bool valid() const;
bool valid(double var_threshold,double age_threshold,double invDepth_max,
double invDepth_min) const;	 
3.3 SmartGrid.h
	1.模板类SmartGrid
		成员：std::vector<std::vector<T *> *> _grid;  二维指针矩阵，指向T
			  std::list<T>_elements;               存放T的列表，指针_grid中
			  T  _invalid;
		函数：
			构造函数，初始化_grid为指定大小的二维指针矩阵
			析构函数，new的内存为存储T指针的向量，只需要delete _grid中元素
			重载运算符=，重载（）取出T，指针为null则取_invalid（at()函数同）
			set(size_t row, size_t col, const T &value)，
在_elements和_grid中插入新值/替换旧值
reset()
			  设置_grid中所有指针为null，清空_elements
			clear() 
在_elements和_grid中删除invalid的值
			clean(double var_threshold,double age_threshold,double range_max,double range_min)
在_elements和_grid中删除invalid的值（DepthPoint重载函数）
			erode(size_t radius, size_t border, double ratio)
			exists()
			  _grid中指针是否为空
			remove()
			  输入迭代器，置其对应_grid指针为null，_elements删除该元素，返回迭代器			
resize()
			  先调用reset(),然后利用输入的行数，增加二维指针矩阵大小
			erode(size_t radius, size_t border, double ratio)
			   给定半径，border，ratio，以最初的深度点分布判断，如果周围空点太多，剔除
			dilate(size_t radius) 
			   给定半径，在周围填充初始化的深度点
			getNeighbourhood
			   将指针不为null且valid的邻近点的指针存入向量
rows()，cols()得到二维指针矩阵的行列
			size()，返回_elements的size，即二维指针矩阵有效个数
			begin()，end()，返回_elements的迭代器
3.4 DepthMap.h
	包含container中的DepthPoint.h和SmartGrid.h，tools中的utils.h
	using DepthMap = SmartGrid<DepthPoint>
	1.结构体DepthFrame
		成员：DepthMap::Ptr dMap_;
  			  size_t id_;
              Transformation T_world_frame_;  
		函数：赋值和清空的的接口
3.5 CameraSystem.h CameraSystem.cpp
	1.类PerspectiveCamera，维护一个相机的标定信息和3D2D转换
		成员：  size_t width_, height_       像平面大小
  std::string cameraName_	 相机名称
  std::string distortion_model_  模型名称
	
标定得到的矩阵D、K、R
  Eigen::Matrix<double, 4, 1> D_
  Eigen::Matrix3d K_;
  Eigen::Matrix3d RectMat_
  Eigen::Matrix<double, 3, 4> P_

  Eigen::Matrix2Xd precomputed_rectified_points_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::MatrixXi UndistortRectify_mask_;
		函数：setIntrinsicParameters，
为标定得到的矩阵赋值，输入为vector
			  preComputeRectifiedCoordinate，
根据distortion_model_将原始坐标转为rect坐标 precomputed_rectified_points_，同时矫正图像得到undistort_map1_, undistort_map2_，UndistortRectify_mask_
			  getRectifiedUndistortedCoordinate，输入原始坐标，得到矫正后坐标
 
			  cam2World
			  world2Cam
	2.类CameraSystem，组成双目
		成员： PerspectiveCamera::Ptr cam_left_ptr_, cam_right_ptr_ 两个相机的智能指针
			   Eigen::Matrix<double, 3, 4> T_right_left_
			   double baseline_
		函数：  loadCalibInfo，从yaml文件读取两个相机的标定信息
computeBaseline，
 
3.6 TimeSurfaceObservation.h
包含tools的TicToc.h,utils.h
1.结构体TimeSurfaceObservation
	成员：Eigen::MatrixXd TS_left_, TS_right_;
  		  Eigen::MatrixXd TS_blurred_left_;
  		  Eigen::MatrixXd TS_negative_left_;
  		  cv_bridge::CvImagePtr cvImagePtr_left_, cvImagePtr_right_;
  		  Transformation tr_;
  		  Eigen::MatrixXd dTS_du_left_, dTS_dv_left_;
  		  Eigen::MatrixXd dTS_negative_du_left_, dTS_negative_dv_left_;
  		  size_t id_;
	函数：
构造函数：
初始化tr_和id_，cv转eigen生成TS_left_, TS_right_，计算左图上x、y方向sobel，生成dTS_du_left_, dTS_dv_left_
重载构造函数：
不初始化tr_的版本，初始化cvImagePtr_left_ = left;
cvImagePtr_right_ = right，其他相同
			 GaussianBlurTS(size_t kernelSize)
				cv::GaussianBlur后存入TS_right_ TS_left_
			 getTimeSurfaceNegative(size_t kernelSize)
				有核先cv::GaussianBlur后存入TS_blurred_left_
				后减去得到负时间表面
			 computeTsNegativeGrad()
			计算负时间表面的sobel，存入dTS_negative_du_left_, dTS_negative_dv_left_
setTransformation() 
用于设置tr_
isEmpty() 
	判左右TS是否空
2.结构体ROSTimeCmp，重载一个括号运算符，用于比较ros::Time
3. 
using TimeSurfaceHistory = std::map<ros::Time, TimeSurfaceObservation, ROSTimeCmp>
using StampedTimeSurfaceObs = std::pair<ros::Time, TimeSurfaceObservation>
		函数：
			TSHistory_lower_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
			TSHistory_upper_bound(TimeSurfaceHistory &ts_history, ros::Time &t)
			按时间从大到小，找到两个边界
3.7 EventMatchPair.h
包含tools的utils.h和container的CameraSystem.h和TimeSurfaceObservation.h
1.结构体EventMatchPair
		成员 
// raw event coordinate
  	Eigen::Vector2d x_left_raw_;
  	// rectified_event coordinate (left, right)
  	Eigen::Vector2d x_left_, x_right_;
  	// timestamp
  	ros::Time t_;
  	// pose of virtual view (T_world_virtual)
  	Transformation trans_;
  	// inverse depth
  	double invDepth_;
  	// match cost
  	double cost_;
  	// gradient (left)
  	double gx_, gy_;
  	// disparity
  	double disp_;
 
四、core
	4.1. DepthProblem.h DepthProblem.cpp
		1.结构体DepthProblemConfig
			成员：
size_t  patchSize_X_, patchSize_Y_;
std::string LSnorm_;    l2 zncc
double td_nu_;
double td_scale_;
double td_scaleSquared_; // td_scale_^2
double td_stdvar_;      // sigma
size_t MAX_ITERATION_;
		2.类DepthPorblem 继承自OptimizationFunctor<double>
			成员：
				  CameraSystem::Ptr camSysPtr_;          双目相机的指针
  DepthProblemConfig::Ptr dpConfigPtr_;    DepthProblemConfig的指针
  Eigen::Vector2d coordinate_;

  Eigen::Matrix<double,4,4> T_world_virtual_;
std::vector<Eigen::Matrix<double, 3, 4> >  vT_left_virtual_;
StampedTimeSurfaceObs* pStampedTsObs_;  
//std::pair<ros::Time, TimeSurfaceObservation>
			函数：
setProblem   赋值coordinate_，T_world_virtual_ ，pStampedTsObs_ = pStampedTsObs。计算得到left到virtual的变换T
warping     将virtual上点重投影到stereo observation上
patchInterpolation
重载括号运算 给定向量的深度和frec，通过warping，patchinerpolation，通过给定LSnorm 比如zncc计算两个patch的相关性
4.2 DepthProblemSolver.h
	引用，core中的DepthProblem.h、container中EventMatchPair.h、optimization中OptimizationFunctor.h
	4.3. DepthRegularization.h 
	4.4. DepthRegularization.h 
	4.5. DepthFusion.h

	4.6. EventBM.h （bestmatch） 通过zncc/  得到双目事件匹配对
		   成员：
 
  				StampedTimeSurfaceObs* pStampedTsObs_; 	pair指针 t+TS
  				StampTransformationMap * pSt_map_;	    pair指针 t+TF
  				std::vector<dvs_msgs::Event*> vEventsPtr_;   事件指针的向量
  				std::vector<std::pair<size_t, size_t> > vpDisparitySearchBound_;

			    CameraSystem::Ptr camSysPtr_;   双目相机的指针
  				Sobel sb_; 
  size_t NUM_THREAD_;
  bool bSmoothTS_;
  //parameters
  size_t patch_size_X_;size_t patch_size_Y_;
  size_t min_disparity_;size_t max_disparity_;
  size_t step_;
  double ZNCC_Threshold_;
  double ZNCC_MAX_;
  bool bUpDownConfiguration_;
		//for test
size_t coarseSearchingFailNum_, 
fineSearchingFailNum_,
infoNoiseRatioLowNum_;
		  函数：
			   resetParameters
			   createMatchProblem  生成带时间戳的TS、TF还有事件流，同时生成相应
大小的DisparitySearchBound
			   isValidPatch，通过patch和x计算左上角和右下角点，满足不在图片边缘的条件返回true
			   zncc_cost 输入左右TS，调用tools::normalizepatch,输出zncc cost
cost = 0.5 * (1 - (patch_left_normalized.array() * patch_right_normalized.array()).sum() / (patch_left.rows() * patch_left.cols()));
			   epipolarSearching，
					输入，搜索初始位置、结束位置、搜索步长
					输出，min_cost, bestMatch, size_t& bestDisp, Eigen::MatrixXd& patch_dst
			   match_an_event, 输入事件和视差界限，通过epipolarSearching输出事件对
			   match_all_SingleThread
match，针对jobs
  			   match_all_HyperThread，建立jobs，加入线程中，最后融合结果。
	4.7. RegProblemLM.h
	

