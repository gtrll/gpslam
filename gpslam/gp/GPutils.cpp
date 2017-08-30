/**
 *  @file  GPutils.cpp
 *  @brief GP utils, calculation of Qc, Q, Lamda matrices etc.
 *  @author Xinyan Yan, Jing Dong
 *  @date Qct 26, 2015
 **/

#include <gpslam/gp/GPutils.h>

using namespace gtsam;


namespace gpslam {

/* ************************************************************************** */
Matrix getQc(const SharedNoiseModel& Qc_model) {
  noiseModel::Gaussian *Gassian_model =
      dynamic_cast<noiseModel::Gaussian*>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

} // namespace gtsam


