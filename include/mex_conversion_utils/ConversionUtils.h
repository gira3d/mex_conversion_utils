#pragma once

#include "mex.h"
#include <Eigen/Geometry>

namespace conversion_utils
{
  /*
   * Convert a pointcloud to Eigen::MatrixXf
   */
  inline void matlabToEigen( const mxArray* prhs[],
                       const unsigned int index,
                       Eigen::MatrixXf &output )
  {
    unsigned int nrows = mxGetM(prhs[index]);
    unsigned int ncols = mxGetN(prhs[index]);

    output = Eigen::MatrixXf::Zero( nrows, ncols );

    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
          output(j,i) = *(mxGetPr(prhs[index])+count);
  }


  /*
   * Convert a pointcloud to Eigen::MatrixXf
   */
  inline void matlabToVecXf( const mxArray* prhs[],
                             const unsigned int index,
                             Eigen::VectorXf& output )
  {
    unsigned int nrows = mxGetM(prhs[index]);
    unsigned int ncols = mxGetN(prhs[index]);

    output = Eigen::VectorXf::Zero( std::max(ncols, nrows) );

    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
        output(count) = *(mxGetPr(prhs[index])+count);
  }

  /*
   * Convert a pointcloud to Eigen::MatrixXd
   */
  inline void matlabToVecXd( const mxArray* prhs[],
                             const unsigned int index,
                             Eigen::VectorXd& output )
  {
    unsigned int nrows = mxGetM(prhs[index]);
    unsigned int ncols = mxGetN(prhs[index]);
    unsigned int nelem = nrows*ncols;

    output = Eigen::VectorXd::Zero( std::max(ncols, nrows) );

    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
        output(count) = *(mxGetPr(prhs[index])+count);
  }


  /*
   * Convert a pointcloud to Eigen::MatrixXf
   */
  inline void matlabToEigen( const mxArray* prhs[],
                             const unsigned int index,
                             Eigen::MatrixXd &output )
  {
    unsigned int nrows = mxGetM(prhs[index]);
    unsigned int ncols = mxGetN(prhs[index]);

    output = Eigen::MatrixXd::Zero( nrows, ncols );

    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
        output(j,i) = *(mxGetPr(prhs[index])+count);
  }

  /*
   * Convert matlab matrix to Eigen::Matrix3f
   */
  inline void matlabToEigenMat3f( const mxArray* prhs[],
                             const unsigned int index,
                             Eigen::Matrix3f &output )
  {
    unsigned int nrows = mxGetM(prhs[index]);
    unsigned int ncols = mxGetN(prhs[index]);

    if ((nrows != 3) && (ncols !=3))
    {
      std::cerr << "ERROR: matrix should be 3x3 but was " << nrows << "x" << ncols << " instead" << std::endl;
      return;
    }

    output = Eigen::Matrix3f::Zero( 3, 3 );

    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
        output(j,i) = *(mxGetPr(prhs[index])+count);
  }




  /*
   * matlabToEigenVec3d
   *
   * Convert a 3d matlab point to Eigen::Vector3d
   * inputs:
   *        prhs: array of inputs from user
   *        index: index of prhs to pull data from
   *        output: Eigen::Vector3d containing data
   */
  inline void matlabToEigenVec3d( const mxArray* prhs[],
                                  const unsigned int index,
                                  Eigen::Vector3d &output )
  {
    uint8_t nrows = mxGetM(prhs[index]);
    uint8_t ncols = mxGetN(prhs[index]);

    if ( !( ( (nrows == 3) && (ncols == 1) ) ^
            ( (nrows == 1) && (ncols == 3) ) ) )
      mexWarnMsgTxt("matlabToEigenVec3d: data incorrect size.");

    output = Eigen::Vector3d::Zero();
    output(0) = *(mxGetPr(prhs[index]));
    output(1) = *(mxGetPr(prhs[index])+1);
    output(2) = *(mxGetPr(prhs[index])+2);
  }

  /*
   * matlabToEigenVec3f
   *
   * Convert a 3d matlab point to Eigen::Vector3d
   * inputs:
   *        prhs: array of inputs from user
   *        index: index of prhs to pull data from
   *        output: Eigen::Vector3d containing data
   */
  inline void matlabToEigenVec3f( const mxArray* prhs[],
                                  const unsigned int index,
                                  Eigen::Vector3f &output )
  {
    uint8_t nrows = mxGetM(prhs[index]);
    uint8_t ncols = mxGetN(prhs[index]);

    if ( !( ( (nrows == 3) && (ncols == 1) ) ^
            ( (nrows == 1) && (ncols == 3) ) ) )
      mexWarnMsgTxt("matlabToEigenVec3d: data incorrect size.");

    output = Eigen::Vector3f::Zero();
    output(0) = *(mxGetPr(prhs[index]));
    output(1) = *(mxGetPr(prhs[index])+1);
    output(2) = *(mxGetPr(prhs[index])+2);
  }

  /*
   *
   */
  inline void matlabToEigenVec4d( const mxArray* prhs[],
                                  const unsigned int index,
                                  Eigen::Vector4d& output)
  {
    uint8_t nrows = mxGetM(prhs[index]);
    uint8_t ncols = mxGetN(prhs[index]);

    if ( !( ( (nrows == 4) && (ncols == 1) ) ^
            ( (nrows == 1) && (ncols == 4) ) ))
      mexWarnMsgTxt("matlabToEigenVec4d: data incorrect size.");

    output = Eigen::Vector4d::Zero();
    output(0) = *(mxGetPr(prhs[index]));
    output(1) = *(mxGetPr(prhs[index])+1);
    output(2) = *(mxGetPr(prhs[index])+2);
    output(3) = *(mxGetPr(prhs[index])+3);
  }

  /*
   * Convert an std::vector to matlab's format
   */
  template<class T>
  inline void stdVectorToMATLAB(const std::vector<T>& vec,
                                const unsigned int index,
                                mxArray *plhs[])
  {
    plhs[index] = mxCreateDoubleMatrix((mwSize) vec.size(), 1, mxREAL);
    for (unsigned int i = 0; i < vec.size(); ++i)
      *(mxGetPr(plhs[index])+i) = ((double)vec[i]);
  }

  /*
   * Convert an std::vector to matlab's format
   */
  template<class T>
    inline bool matlabToStdVector(const mxArray *prhs[],
                                  const unsigned int& index,
                                  std::vector<T>& vec)

  {
    uint32_t nrows = mxGetM(prhs[index]);
    uint32_t ncols = mxGetN(prhs[index]);

    if (ncols == 1)
    {
      for (uint32_t i = 0; i < nrows; ++i)
        vec.push_back(*(mxGetPr(prhs[index])+i));
    }
    else
    {
      std::cerr << "ERROR: input matrix not 1xn" << std::endl;
      return false;
    }
    return true;
  }

  /*
   * Convert an eigen matrix to matlab's format
   */
  template<typename T>
    inline void eigenToMATLAB(const Eigen::Matrix<T,-1,-1>& matrix,
                            const unsigned int index,
                            mxArray *plhs[])
  {
    const int nrows = matrix.rows();
    const int ncols = matrix.cols();
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    for (int i = 0, count = 0; i < ncols; ++i)
      for (int j = 0; j < nrows; ++j, ++count)
        *(mxGetPr(plhs[index])+count) = matrix(j,i);
  }

  template<typename T>
    inline void eigenToMATLAB(const Eigen::Matrix<T,-1,1>& matrix,
                              const unsigned int index,
                              mxArray *plhs[])
  {
    const int nrows = matrix.rows();
    const int ncols = matrix.cols();
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    for (int i = 0, count = 0; i < ncols; ++i)
      for (int j = 0; j < nrows; ++j, ++count)
        *(mxGetPr(plhs[index])+count) = matrix(j,i);
  }

  /*
   * Convert an eigen matrix to matlab's format
   */
  inline void eigenToMATLAB(const Eigen::MatrixXf& matrix,
                            const unsigned int index,
                            mxArray *plhs[])
  {
    const int nrows = matrix.rows();
    const int ncols = matrix.cols();
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    for (int i = 0, count = 0; i < ncols; ++i)
      for (int j = 0; j < nrows; ++j, ++count)
          *(mxGetPr(plhs[index])+count) = matrix(j,i);
  }

  /*
   * Convert an eigen Double matrix to matlab's format
   */

  inline void eigenDoubleToMATLAB(const Eigen::MatrixXd& matrix,
                            const unsigned int index,
                            mxArray *plhs[])
  {
    const int nrows = matrix.rows();
    const int ncols = matrix.cols();
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
          *(mxGetPr(plhs[index])+count) = matrix(j,i);
  }


  /*
   * Convert an eigen vectorXd to matlab's format
   * inputs:
   *        vec: Eigen::VectorXd input
   *        index: index of plhs to insert data into
   *        output: 3x1 mxArray
   *
   */

  inline void eigenVecXdToMATLAB(const Eigen::VectorXd& vec,
                            const unsigned int index,
                            mxArray *plhs[])
  {
    const int nrows = vec.rows();
    const int ncols = 1;
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    for (unsigned int i = 0, count = 0; i < ncols; ++i)
      for (unsigned int j = 0; j < nrows; ++j, ++count)
        *(mxGetPr(plhs[index])+count) = vec(j,i);
  }

  /*
   * Convert an eigen vector3d to matlab's format
   * inputs:
   *        vec: Eigen::Vector3d input
   *        index: index of plhs to insert data into
   *        output: 3x1 mxArray
   *
   */
  inline void eigenVec3dToMATLAB(const Eigen::Vector3d& vec,
                            const unsigned int index,
                            mxArray *plhs[])
  {
    const int nrows = 3;
    const int ncols = 1;
    plhs[index] = mxCreateDoubleMatrix((mwSize) nrows, ncols, mxREAL);
    *(mxGetPr(plhs[index])) = vec(0);
    *(mxGetPr(plhs[index])+1) = vec(1);
    *(mxGetPr(plhs[index])+2) = vec(2);
  }



  /*
   * Convert a scalar to MATLAB's format
   * inputs:
   *        vec: Eigen::Vector3d input
   *        index: index of plhs to insert data into
   *        output: 3x1 mxArray
   *
   */
  template <typename T>
  inline void scalarToMATLAB(const T& d,
                             const unsigned int index,
                             mxArray *plhs[])
  {
    plhs[index] = mxCreateDoubleMatrix((mwSize) 1, 1, mxREAL);
    *(mxGetPr(plhs[index])) = d;
  }

  inline double matlabToScalar(const mxArray *prhs[],
                               const unsigned int idx)
  {
    return mxGetScalar(prhs[idx]);
  }

  template<typename T>
    inline Eigen::Transform<T, 3, Eigen::Affine> matlabToAffine3(const mxArray *prhs[],
                                                                 const unsigned int idx)
  {
    Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T,3,3>::Zero(3,3);
    Eigen::Matrix<T, 3, 1> t = Eigen::Matrix<T,3,1>::Zero(3,1);

    uint32_t count = 0;
    for (uint32_t i = 0; i < 3; ++i)
      for (uint32_t j = 0; j < 3; ++j, ++count)
      {
        R(j,i) = *(mxGetPr(prhs[idx])+count);
      }

    t << *(mxGetPr(prhs[idx])+count), *(mxGetPr(prhs[idx])+count+1), *(mxGetPr(prhs[idx])+count+2);

    Eigen::Transform<T, 3, Eigen::Affine> Tr;
    Tr.linear() = R;
    Tr.translation() = t;
    return Tr;
  }

  /*
   * Convert an eigen matrix to Affine3f
   */
  inline void toAffine3f(const Eigen::MatrixXf& To,
                         Eigen::Affine3f& Tn)
  {
    for (uint32_t i = 0; i < 4; ++i)
      for (uint32_t j = 0; j < 4; ++j)
        Tn(i,j) = To(i,j);
  }
}
