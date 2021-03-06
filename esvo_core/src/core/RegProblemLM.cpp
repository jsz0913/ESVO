#include <esvo_core/core/RegProblemLM.h>
#include <esvo_core/tools/cayley.h>
#include <thread>
#include <algorithm>

namespace esvo_core
{
namespace core
{
RegProblemLM::RegProblemLM(
  const CameraSystem::Ptr& camSysPtr,
  const RegProblemConfig::Ptr& rpConfig_ptr,
  size_t numThread):
  optimization::OptimizationFunctor<double>(6,0),
  camSysPtr_(camSysPtr),
  rpConfigPtr_(rpConfig_ptr),
  NUM_THREAD_(numThread),
  bPrint_(false)
{
  patchSize_ = rpConfigPtr_->patchSize_X_ * rpConfigPtr_->patchSize_Y_;
  computeJ_G(Eigen::Matrix<double,6,1>::Zero(), J_G_0_);
}

void RegProblemLM::setProblem(RefFrame* ref, CurFrame* cur, bool bComputeGrad)
{
  ref_ = ref;
  cur_ = cur;
  // TS对应depthFrame对应的refFrame 当前TS对应CurFrame
  // cur_.tr_ = Transformation(T_world_cur_)
  T_world_ref_  = ref_->tr_.getTransformationMatrix();
  T_world_left_ = cur_->tr_.getTransformationMatrix();
  
  // left到ref 
  Eigen::Matrix4d T_ref_left = T_world_ref_.inverse() * T_world_left_;
  
  // 初始值
  R_ = T_ref_left.block<3,3>(0,0);
  t_ = T_ref_left.block<3,1>(0,3);
  
  // ref
  Eigen::Matrix3d R_world_ref = T_world_ref_.block<3,3>(0,0);
  Eigen::Vector3d t_world_ref = T_world_ref_.block<3,1>(0,3);

  // load ref's pointcloud tp vResItem，把ref中世界坐标系点云转换到ref坐标系，存入ResItems_中
  ResItems_.clear();
  numPoints_ =ref_->vPointXYZPtr_.size();
  if(numPoints_ > rpConfigPtr_->MAX_REGISTRATION_POINTS_)
    numPoints_ = rpConfigPtr_->MAX_REGISTRATION_POINTS_;
  ResItems_.resize(numPoints_);
  if(bPrint_)
    LOG(INFO) << "num points: " << numPoints_;

  for(size_t i = 0; i < numPoints_; i++)
  {
    bool bStochasticSampling = true;
    // 随机交换
    if (bStochasticSampling)
      std::swap(ref->vPointXYZPtr_[i], ref->vPointXYZPtr_[i + rand() % (ref->vPointXYZPtr_.size() - i)]);
    Eigen::Vector3d p_tmp((double) ref->vPointXYZPtr_[i]->x,
                          (double) ref->vPointXYZPtr_[i]->y,
                          (double) ref->vPointXYZPtr_[i]->z);
    // world to ref
    // 同乘转置可以看出
    Eigen::Vector3d p_cam = R_world_ref.transpose() * (p_tmp - t_world_ref);
    // initialize用于遍历赋值
    ResItems_[i].initialize(p_cam(0), p_cam(1), p_cam(2));//, var);
  }
  
  // for stochastic sampling
  numBatches_ = std::max(ResItems_.size() / rpConfigPtr_->BATCH_SIZE_, (size_t)1);
  *********************************************************************************************
  // load cur's info
  pTsObs_ = cur->pTsObs_;
  // 高斯blur 后 计算负时间表面存入
  pTsObs_->getTimeSurfaceNegative(rpConfigPtr_->kernelSize_);
  if(bComputeGrad)
    // 计算负时间表面的sobel 分别存入，rpType_ == REG_ANALYTICAL
    pTsObs_->computeTsNegativeGrad();
  
  // set fval dimension 函数值维度
  resetNumberValues(numPoints_ * patchSize_);
  if(bPrint_)
    LOG(INFO) << "RegProblemLM::setProblem succeeds.";
}

void RegProblemLM::setStochasticSampling(size_t offset, size_t N)
{
  ResItemsStochSampled_.clear();
  ResItemsStochSampled_.reserve(N);
  for(size_t i = 0;i < N;i++)
  {
    // offset+i ... N < ResItems_.size()
    if(offset + i >= ResItems_.size())
      break;
    ResItemsStochSampled_.push_back(ResItems_[offset + i]);
  }
  numPoints_ = ResItemsStochSampled_.size();
  resetNumberValues(numPoints_ * patchSize_);
  if(bPrint_)
  {
    LOG(INFO) << "offset: " << offset;
    LOG(INFO) << "N: " << N;
    LOG(INFO) << "ResItems_.size: " << ResItems_.size();
    LOG(INFO) << "ResItemsStochSampled_.size: " << ResItemsStochSampled_.size();
  }
}

int RegProblemLM::operator()(const Eigen::Matrix<double,6,1>& x, Eigen::VectorXd& fvec) const
{
  // calculate the warping transformation (T_cur_ref))
  // 
  Eigen::Matrix4d T_warping = Eigen::Matrix4d::Identity();
  getWarpingTransformation(T_warping, x);

  // warp and calculate the residual
  std::vector<Job> jobs(NUM_THREAD_);
  for(size_t i = 0;i < NUM_THREAD_;i++)
  {
    jobs[i].pvRI_ = const_cast<ResidualItems*>(&ResItemsStochSampled_);
    jobs[i].pTsObs_ = const_cast<TimeSurfaceObservation*>(pTsObs_);
    jobs[i].T_left_ref_ = const_cast<Eigen::Matrix4d*>(&T_warping);
    jobs[i].i_thread_ = i;
  }

  std::vector<std::thread> threads;
  for(size_t i = 0; i < NUM_THREAD_; i++)
    threads.emplace_back(std::bind(&RegProblemLM::thread, this, jobs[i]));
  for( auto& thread : threads)
    if(thread.joinable())
      thread.join();

  // assign the reweighted residual to fvec
  // fvec  (r.size * patchsize,1)
  // fjac  (r.size * patchsize , 6)
  if(strcmp(rpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
  {
    for(size_t i = 0; i < ResItemsStochSampled_.size(); i++)
    {
      ResidualItem & ri = const_cast<ResidualItem&>(ResItemsStochSampled_[i]);
      // segment切片，向量第 i * ri.residual_.size() 开始，ri.residual_.size()个
      fvec.segment(i * ri.residual_.size(), ri.residual_.size()) = ri.residual_;// / sqrt(var);
    }
  }
  if(strcmp(rpConfigPtr_->LSnorm_.c_str(), "Huber") == 0)
  {
    for(size_t i = 0; i < ResItemsStochSampled_.size(); i++)
    {
      ResidualItem & ri = const_cast<ResidualItem&>(ResItemsStochSampled_[i]);
      double irls_weight = 1.0;
      if(ri.residual_(0) > rpConfigPtr_->huber_threshold_)
        irls_weight = rpConfigPtr_->huber_threshold_ / ri.residual_(0);
      //这个不也应该这样么？
      fvec[i] = sqrt(irls_weight) * ri.residual_(0);
    }
  }
//  LOG(INFO) << "assign weighted residual ..............";
  return 0;
}

void
RegProblemLM::thread(Job& job ) const
{
  // load info from job
  ResidualItems & vRI = *job.pvRI_;
  const TimeSurfaceObservation & TsObs = *job.pTsObs_;
  const Eigen::Matrix4d & T_left_ref = *job.T_left_ref_;
  size_t i_thread = job.i_thread_;
  size_t numPoint = vRI.size();
  size_t wx = rpConfigPtr_->patchSize_X_;
  size_t wy = rpConfigPtr_->patchSize_Y_;
  size_t residualDim = wx * wy;

  // calculate point-wise spatio-temporal residual
  // the residual can be either a scalr or a vector, up to the residualDim.
  // 老一套，i跳着取
  for(size_t i = i_thread; i < numPoint; i+= NUM_THREAD_)
  {
    //取出一个ResidualItem，重投影，patchInterpolation，将结果存入residual_，否则255
    ResidualItem & ri = vRI[i];
    ri.residual_ = Eigen::VectorXd(residualDim);
    Eigen::Vector2d x1_s;
    if(!reprojection(ri.p_, T_left_ref, x1_s))
      ri.residual_.setConstant(255.0);
    else
    {
      Eigen::MatrixXd tau1;
      
      if(patchInterpolation(TsObs.TS_negative_left_, x1_s, tau1))
      {
        for(size_t y = 0; y < wy; y++)
          for(size_t x = 0; x < wx; x++)
          {
            size_t index = y * wx + x;
            //residual_即TSnegative的值
            ri.residual_[index] = tau1(y,x);
          }
      }
      else
        ri.residual_.setConstant(255.0);
    }
  }
}

int RegProblemLM::df(const Eigen::Matrix<double,6,1>& x, Eigen::MatrixXd& fjac) const
{
  if(x != Eigen::Matrix<double,6,1>::Zero())
  {
    LOG(INFO) << "The Jacobian is not evaluated at Zero !!!!!!!!!!!!!";
    exit(-1);
  }
  //m_values
  fjac.resize(m_values, 6);

  // J_x = dPi_dT * dT_dInvPi * dInvPi_dx
  // 对x的雅各比 = dpi/dT * dT/dinvPi * dinvPi/dx
  // 反投影-T转换-投影

  Eigen::Matrix3d dT_dInvPi = R_.transpose();// Explaination for the transpose() can be found below.
  Eigen::Matrix<double,3,2> dInvPi_dx_constPart;
  
  // ref 上x 反投影到 invPi 转换 T = R_.T*invPi+t_ 投影到 cur 上的pi
  // x -> invPi   world2Cam   u=(p(0,0)*x+p(0,2)*z)/z v=(p(1,1)*y+p(1,2)*z)/z   
  dInvPi_dx_constPart.setZero();
  dInvPi_dx_constPart(0,0) = 1.0 / camSysPtr_->cam_left_ptr_->P_(0,0);
  dInvPi_dx_constPart(1,1) = 1.0 / camSysPtr_->cam_left_ptr_->P_(1,1);
  //dInvPi_dx_constPart
  // 1/P(0,0)     0       
  //   0      1/P(1,1)   
  //   0          0
  
  Eigen::Matrix<double,3,2> J_constPart = dT_dInvPi * dInvPi_dx_constPart;

  // J_theta = dPi_dT * dT_dG * dG_dtheta
  // 对theta的雅各比
  // Assemble the Jacobian without dG_dtheta.
  Eigen::MatrixXd fjacBlock;
  fjacBlock.resize(numPoints_, 12);
  Eigen::MatrixXd fjacTMP(3,6);//FOR Test
 
  Eigen::Matrix4d T_left_ref = Eigen::Matrix4d::Identity();
  T_left_ref.block<3,3>(0,0) = R_.transpose();
  T_left_ref.block<3,1>(0,3) = -R_.transpose() * t_;

  const double P11 = camSysPtr_->cam_left_ptr_->P_(0,0);
  const double P12 = camSysPtr_->cam_left_ptr_->P_(0,1);
  const double P14 = camSysPtr_->cam_left_ptr_->P_(0,3);
  const double P21 = camSysPtr_->cam_left_ptr_->P_(1,0);
  const double P22 = camSysPtr_->cam_left_ptr_->P_(1,1);
  const double P24 = camSysPtr_->cam_left_ptr_->P_(1,3);
  // fjac numPoints 行 见237行
  for(size_t i = 0; i < numPoints_; i++)
  {
    Eigen::Vector2d x1_s;
    const ResidualItem & ri = ResItemsStochSampled_[i];
    if(!reprojection(ri.p_, T_left_ref, x1_s))
      // 对应fvec设置为255
      fjacBlock.row(i) = Eigen::Matrix<double,1,12>::Zero();
    else
    {
      // obtain the exact gradient by bilinear interpolation.
      // 得到投影点对应的梯度 即dI/du 
      Eigen::MatrixXd gx, gy;
      patchInterpolation(pTsObs_->dTS_negative_du_left_, x1_s, gx);
      patchInterpolation(pTsObs_->dTS_negative_dv_left_, x1_s, gy);
      Eigen::Vector2d grad = Eigen::Vector2d(gx(0,0)/8, gy(0,0)/8);//8 is the normalization factor for 3x3 sobel filter.
      // 2*3 像素坐标对相机坐标系
      // u = 【p（0，0）* x + p（0，1）* y + p（0，2）* z】/z
      // v = 【p（1，0）* x + p（1，1）* y + p（2，2）* z】/z
      // 这写法把P整个考虑进去，同时z不为1，反过来求导跟坐标有关
      Eigen::Matrix<double,2,3> dPi_dT;
      dPi_dT.setZero();
      dPi_dT.block<2,2>(0,0) = camSysPtr_->cam_left_ptr_->P_.block<2,2>(0,0) / ri.p_(2);
      const double z2 = pow(ri.p_(2),2);
      dPi_dT(0,2) = -(P11 * ri.p_(0) + P12 * ri.p_(1) + P14) / z2;
      dPi_dT(1,2) = -(P21 * ri.p_(0) + P22 * ri.p_(1) + P24) / z2;

      // assemble dT_dG T应该是在参考帧delta后的点
      Eigen::Matrix<double,3,12> dT_dG;
      dT_dG.setZero();
      dT_dG.block<3,3>(0,0) = ri.p_(0) * Eigen::Matrix3d::Identity();
      dT_dG.block<3,3>(0,3) = ri.p_(1) * Eigen::Matrix3d::Identity();
      dT_dG.block<3,3>(0,6) = ri.p_(2) * Eigen::Matrix3d::Identity();
      dT_dG.block<3,3>(0,9) = Eigen::Matrix3d::Identity();
      /* dT_dG
        diag(x,x,x) diag(y,y,y) diag(z,z,z) diag(1,1,1)
      */
      //                      (1*2)       * (2*3)  *     (3*2)   *     (2*3) * (3*12) *（1*1 z）
      // 最后这个z应该是？
      fjacBlock.row(i) = grad.transpose() * dPi_dT * J_constPart *     dPi_dT * dT_dG * ri.p_(2);
      // J_constPart = dT_dInvPi * dInvPi_dx_constPart
      // ri.p_(2) refers to 1/rho_i which is actually coming with dInvPi_dx.
    }
  }
  // assemble with dG_dtheta
  //
  fjac = -fjacBlock * J_G_0_;
  // The explanation for the factor -1 is as follows. The transformation recovered from dThetha
  // is T_right_left (R_, t_). However, the one used for warping is T_left_right (R_.transpose(), -R.transpose() * t).
  // Thus, R_.transpose() is used as dT_dInvPi. Besides, J_theta = dPi_dT * dT_dG' * dG'_dG * dG_dtheta. G'(dtheta) recovers
  // the motion for the warping, namely R_.transpose(), -R.transpose() * t.
  //          /                                 \
  //          | 1 0 0 0 0 0 0 0 0       | 0 0 0 |
  //          | 0 0 0 1 0 0 0 0 0       | 0 0 0 |
  //          | 0 0 0 0 0 0 1 0 0       | 0 0 0 |
  //          | 0 1 0 0 0 0 0 0 0       | 0 0 0 |
  //          | 0 0 0 0 1 0 0 0 0       | 0 0 0 |
  // dG'_dG = | 0 0 0 0 0 0 0 1 0       | 0 0 0 |
  //          | 0 0 1 0 0 0 0 0 0       | 0 0 0 |
  //          | 0 0 0 0 0 1 0 0 0       | 0 0 0 |
  //          | 0 0 0 0 0 0 0 0 1       | 0 0 0 |
  //          | -tx -ty -tz 0 0 0 0 0 0 | -r_{11} -r_{21} -r_{31}|
  //          | 0 0 0 -tx -ty -tz 0 0 0 | -r_{12} -r_{22} -r_{32}|
  //          | 0 0 0 0 0 0 -tx -ty -tz | -r_{13} -r_{23} -r_{33}|
  //          \                                                  / 12 x 12
  // The linearization is performed around dtheta = 0, thus tx = ty = tz = 0, r_{ii} = 1, r_{ij} = 0.
  // dG'_dG * dG_dtheta = -dG_dtheta. This explains where is "-1" from.

  // LOG(INFO) << "fjac:\n" << fjac;
  // LOG(INFO) << "Jacobian Computation takes " << tt.toc() << " ms.";
  return 0;
}

 // 求J_G0：dG_dx，
 // G 应该是R t 的所有元素 同时这应该是在x=0处求导
 // 有点像cayley2rot
void
RegProblemLM::computeJ_G(const Eigen::Matrix<double,6,1>&x, Eigen::Matrix<double,12,6>& J_G)
{
  assert( x.size() == 6 );
  assert( J_G.rows() == 12 && J_G.cols() == 6 );
  double c1, c2, c3, k;
  double c1_sq, c2_sq, c3_sq, k_sq;
  
  c1 = x(0);c2 = x(1);c3 = x(2);
  c1_sq = pow(c1,2); c2_sq = pow(c2,2); c3_sq = pow(c3,2);
  
  k = 1 + pow(c1,2) + pow(c2,2) + pow(c3,2);
  k_sq = pow(k,2);
  
  Eigen::Matrix3d A1, A2, A3;
  // A1
  A1(0,0) = 2*c1 / k  - 2*c1*(1 + c1_sq - c2_sq - c3_sq) / k_sq;
  A1(0,1) = -2*c2 / k - 2*c2*(1 + c1_sq - c2_sq - c3_sq) / k_sq;
  A1(0,2) = -2*c3 / k - 2*c3*(1 + c1_sq - c2_sq - c3_sq) / k_sq;
  A1(1,0) = 2*c2 / k - 4*c1*(c1*c2 + c3) / k_sq;
  A1(1,1) = 2*c1 / k - 4*c2*(c1*c2 + c3) / k_sq;
  A1(1,2) = 2 / k    - 4*c3*(c1*c2 + c3) / k_sq;
  A1(2,0) = 2*c3 / k - 4*c1*(c1*c3 - c2) / k_sq;
  A1(2,1) = -2 / k   + 4*c2*(c1*c3 - c2) / k_sq;
  A1(2,2) = 2*c1 / k - 4*c3*(c1*c3 - c2) / k_sq;
  //A2
  A2(0,0) = 2*c2 / k - 4*c1*(c1*c2 - c3) / k_sq;
  A2(0,1) = 2*c1 / k - 4*c2*(c1*c2 - c3) / k_sq;
  A2(0,2) = -2 / k   - 4*c3*(c1*c2 - c3) / k_sq;
  A2(1,0) = -2*c1 / k - 2*c1*(1 - c1_sq + c2_sq - c3_sq) / k_sq;
  A2(1,1) = 2*c2 / k  - 2*c2*(1 - c1_sq + c2_sq - c3_sq) / k_sq;
  A2(1,2) = -2*c3 / k - 2*c3*(1 - c1_sq + c2_sq - c3_sq) / k_sq;
  A2(2,0) = 2 / k    - 4*c1*(c1 + c2*c3) / k_sq;
  A2(2,1) = 2*c3 / k - 4*c2*(c1 + c2*c3) / k_sq;
  A2(2,2) = 2*c2 / k - 4*c3*(c1 + c2*c3) / k_sq;
  //A3
  A3(0,0) = 2*c3 / k - 4*c1*(c2 + c1*c3) / k_sq;
  A3(0,1) = 2 / k    - 4*c2*(c2 + c1*c3) / k_sq;
  A3(0,2) = 2*c1 / k - 4*c3*(c2 + c1*c3) / k_sq;
  A3(1,0) = -2 / k   - 4*c1*(c2*c3 - c1) / k_sq;
  A3(1,1) = 2*c3 / k - 4*c2*(c2*c3 - c1) / k_sq;
  A3(1,2) = 2*c2 / k - 4*c3*(c2*c3 - c1) / k_sq;
  A3(2,0) = -2*c1 / k - 2*c1*(1 - c1_sq - c2_sq + c3_sq) / k_sq;
  A3(2,1) = -2*c2 / k - 2*c2*(1 - c1_sq - c2_sq + c3_sq) / k_sq;
  A3(2,2) = 2*c3 / k  - 2*c3*(1 - c1_sq - c2_sq + c3_sq) / k_sq;

  Eigen::Matrix3d O33 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d I33 = Eigen::MatrixXd::Identity(3,3);
  //J_G
  //  A1     0(3*3)
  //  A2     0(3*3)
  //  A3     0(3*3)
  // 0(3*3)  1(3*3)
  J_G.block<3,3>(0,0) = A1;  J_G.block<3,3>(0,3) = O33;
  J_G.block<3,3>(3,0) = A2;  J_G.block<3,3>(3,3) = O33;
  J_G.block<3,3>(6,0) = A3;  J_G.block<3,3>(6,3) = O33;
  J_G.block<3,3>(9,0) = O33; J_G.block<3,3>(9,3) = I33;
}

void
RegProblemLM::getWarpingTransformation(
  Eigen::Matrix4d& warpingTransf,
  const Eigen::Matrix<double, 6, 1>& x) const
{
  // 把上一次x结果取出，x是delta值，ref 到 cur
  // 利用R_ t_ 来算增量后 但不更新 
  // To calcuate R_cur_ref, t_cur_ref
  Eigen::Matrix3d R_cur_ref;
  Eigen::Vector3d t_cur_ref;
  // get delta cayley paramters (this corresponds to the delta motion of the ref frame)
  Eigen::Vector3d dc = x.block<3,1>(0,0);
  Eigen::Vector3d dt = x.block<3,1>(3,0);
  // add rotation
  Eigen::Matrix3d dR = tools::cayley2rot(dc);
  // R（W（deltaR）） - C（W(R)）
  // R_:cur to ref   R_.T : ref to cur  这是本来固定的W(R)
  // dR相当于缩短了距离，需要右乘逆
  Eigen::Matrix3d newR = R_.transpose() * dR.transpose();
  //
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(newR, Eigen::ComputeFullU | Eigen::ComputeFullV );
  
  R_cur_ref = svd.matrixU() * svd.matrixV().transpose();
  if( R_cur_ref.determinant() < 0.0 )
  {
    LOG(INFO) << "oops the matrix is left-handed\n";
    exit(-1);
  }
  //负值向量方向相反，之后旋转
  t_cur_ref = -R_cur_ref * ( dt + dR * t_ );
  
  warpingTransf.block<3,3>(0,0) = R_cur_ref;
  warpingTransf.block<3,1>(0,3) = t_cur_ref;
}

void
RegProblemLM::addMotionUpdate(const Eigen::Matrix<double, 6, 1>& dx)
{
  /*
  最开始x设置为0,调用warping得到应用增量后的T，直到收敛才更新R_ t_ ?
  */
  // To update R_, t_
  Eigen::Vector3d dc = dx.block<3,1>(0,0);
  Eigen::Vector3d dt = dx.block<3,1>(3,0);
  // add rotation
  Eigen::Matrix3d dR = tools::cayley2rot(dc);
  // warping中 ref 到 cur
  // 这里转置为 cur 到 ref 因为R_本身是 cur 到 ref
  Eigen::Matrix3d newR = dR * R_;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(newR, Eigen::ComputeFullU | Eigen::ComputeFullV );
  R_ = svd.matrixU() * svd.matrixV().transpose();
  t_ = dt + dR * t_;
}

void RegProblemLM::setPose()
{
  T_world_left_.block<3,3>(0,0) = T_world_ref_.block<3,3>(0,0) * R_;
  T_world_left_.block<3,1>(0,3) = T_world_ref_.block<3,3>(0,0) * t_
                                  + T_world_ref_.block<3,1>(0,3);
  cur_->tr_ = Transformation(T_world_left_);
//  LOG(INFO) << "T_world_ref_\n " << T_world_ref_ << "\n ";
//  LOG(INFO) << "T_world_left_\n " << T_world_left_ << "\n ";
//  LOG(INFO) << "R_\n " << R_ << "\n ";
//  LOG(INFO) << "t_\n " << t_.transpose() << "\n ";
}

Eigen::Matrix4d
RegProblemLM::getPose()
{
  return T_world_left_;
}

bool RegProblemLM::isValidPatch(
  Eigen::Vector2d& patchCentreCoord,
  Eigen::MatrixXi& mask,
  size_t wx,
  size_t wy) const
{
  //第一部分，中心点与边的距离要保持在pathc一半以上
  if (patchCentreCoord(0) < (wx-1)/2 ||
      patchCentreCoord(0) > camSysPtr_->cam_left_ptr_->width_  - (wx-1)/2 - 1||
      patchCentreCoord(1) < (wy-1)/2 ||
      patchCentreCoord(1) > camSysPtr_->cam_left_ptr_->height_ - (wy-1)/2 - 1)
    return false;
  //patch的四个角不能为不黑？？
  if(mask(patchCentreCoord(1)-(wy-1)/2, patchCentreCoord(0)-(wx-1)/2) < 125)
    return false;
  if(mask(patchCentreCoord(1)-(wy-1)/2, patchCentreCoord(0)+(wx-1)/2) < 125)
    return false;
  if(mask(patchCentreCoord(1)+(wy-1)/2, patchCentreCoord(0)-(wx-1)/2) < 125)
    return false;
  if(mask(patchCentreCoord(1)+(wy-1)/2, patchCentreCoord(0)+(wx-1)/2) < 125)
    return false;
  return true;
}

bool RegProblemLM::reprojection(
  const Eigen::Vector3d& p,
  const Eigen::Matrix4d& warpingTransf,
  Eigen::Vector2d &x1_s) const
{
  // transfer to left DVS coordinate
  Eigen::Vector3d p_left =
    warpingTransf.block<3, 3>(0, 0) * p + warpingTransf.block<3, 1>(0, 3);
  camSysPtr_->cam_left_ptr_->world2Cam(p_left, x1_s);

  if(!isValidPatch(x1_s, camSysPtr_->cam_left_ptr_->UndistortRectify_mask_,
                   rpConfigPtr_->patchSize_X_, rpConfigPtr_->patchSize_Y_))
    return false;
  return true;
}

//双线性插值的意义
bool RegProblemLM::patchInterpolation(
  const Eigen::MatrixXd &img,
  const Eigen::Vector2d &location,
  Eigen::MatrixXd &patch,
  bool debug) const
{
  int wx = rpConfigPtr_->patchSize_X_;
  int wy = rpConfigPtr_->patchSize_Y_;
  // compute SrcPatch_UpLeft coordinate and SrcPatch_DownRight coordinate
  // check patch bourndary is inside img boundary
  Eigen::Vector2i SrcPatch_UpLeft, SrcPatch_DownRight;
  // location真实坐标，整数后找整数坐标
  SrcPatch_UpLeft << floor(location[0]) - (wx - 1) / 2, floor(location[1]) - (wy - 1) / 2;
  SrcPatch_DownRight << floor(location[0]) + (wx - 1) / 2, floor(location[1]) + (wy - 1) / 2;
  //注意坐标是x对应cols
  if (SrcPatch_UpLeft[0] < 0 || SrcPatch_UpLeft[1] < 0)
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 1: " << SrcPatch_UpLeft.transpose();
    }
    return false;
  }
  if (SrcPatch_DownRight[0] >= img.cols() || SrcPatch_DownRight[1] >= img.rows())
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 2: " << SrcPatch_DownRight.transpose();
    }
    return false;
  }

  // compute q1 q2 q3 q4 周围四个坐标点？
  Eigen::Vector2d double_indices;
  double_indices << location[1], location[0];
  //double_indices： yd xd
  //lower_indices： yi   xi
  //upper_indices： yi+1 xi+1
  std::pair<int, int> lower_indices(floor(double_indices[0]), floor(double_indices[1]));
  std::pair<int, int> upper_indices(lower_indices.first + 1, lower_indices.second + 1);
  //q1：xi + 1  -  xd 右下和中间的x 差值
  //q2：xd      -  xi 左上和中间的x 差值
  //q3：yi+1    -  yd 右下和中间的y 差值
  //q4：yd      -  yi 左上和中间的y 差值
  double q1 = upper_indices.second - double_indices[1];
  double q2 = double_indices[1] - lower_indices.second;
  double q3 = upper_indices.first - double_indices[0];
  double q4 = double_indices[0] - lower_indices.first;

  // extract Src patch, size (wy+1) * (wx+1) 最终是从左上角开始取 (wy+1) * (wx+1)  为了偶数？
  int wx2 = wx + 1;
  int wy2 = wy + 1;
  if (SrcPatch_UpLeft[1] + wy >= img.rows() || SrcPatch_UpLeft[0] + wx >= img.cols())
  {
    if(debug)
    {
      LOG(INFO) << "patchInterpolation 3: " << SrcPatch_UpLeft.transpose()
                << ", location: " << location.transpose()
                << ", floor(location[0]): " << floor(location[0])
                << ", (wx - 1) / 2: " << (wx - 1) / 2
                << ", ans: " << floor(location[0]) - (wx - 1) / 2
                << ", wx: " << wx << " wy: " << wy
                << ", img.row: " << img.rows() << " img.col: " << img.cols();
    }
    return false;
  }
  // 1对应rows对应y
  // 左上取一个
  Eigen::MatrixXd SrcPatch = img.block(SrcPatch_UpLeft[1], SrcPatch_UpLeft[0], wy2, wx2);

  // Compute R, size (wy+1) * wx.
  Eigen::MatrixXd R;
  R = q1 * SrcPatch.block(0, 0, wy2, wx) + q2 * SrcPatch.block(0, 1, wy2, wx);

  // Compute F, size wy * wx.
  patch = q3 * R.block(0, 0, wy, wx) + q4 * R.block(1, 0, wy, wx);
  return true;
}

}
}
