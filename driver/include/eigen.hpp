#pragma once

#include "config.h"
#include "ccommon.h"

#ifdef USE_EIGEN
  #include <Eigen/Core> 

    inline auto _to(XVec3 const& m) { return Eigen::Map<Eigen::Matrix<Float,3,1> const>(m.data); }
    inline auto _to(XVec3 & m) { return Eigen::Map<Eigen::Matrix<Float,3,1>>(m.data); }

    inline auto _to(XVec2 const& m) { return Eigen::Map<Eigen::Matrix<Float,2,1> const>(m.data); }
    inline auto _to(XVec2 & m) { return Eigen::Map<Eigen::Matrix<Float,2,1>>(m.data); }

    inline auto _to(XVec3 const* m) { return Eigen::Map<Eigen::Matrix<Float,3,1> const>(m->data); }
    inline auto _to(XVec3 * m) { return Eigen::Map<Eigen::Matrix<Float,3,1>>(m->data); }

    inline auto _to(XVec2 const* m) { return Eigen::Map<Eigen::Matrix<Float,2,1> const>(m->data); }
    inline auto _to(XVec2 * m) { return Eigen::Map<Eigen::Matrix<Float,2,1>>(m->data); }

    inline auto _to(XVec7 const& m) { return Eigen::Map<Eigen::Matrix<Float,7,1> const>(m.data); }
    inline auto _to(XVec7 & m) { return Eigen::Map<Eigen::Matrix<Float,7,1>>(m.data); }

    inline auto _to(XVec9 const& m) { return Eigen::Map<Eigen::Matrix<Float,9,1> const>(m.data); }
    inline auto _to(XVec9 & m) { return Eigen::Map<Eigen::Matrix<Float,9,1>>(m.data); }

    inline auto _to(XMat26 const& m) { return Eigen::Map<Eigen::Matrix<Float,6,2> const>(m.data).transpose(); }
    inline auto _to(XMat26 & m) { return Eigen::Map<Eigen::Matrix<Float,6,2>>(m.data).transpose(); }

    inline auto _to(XMat33 const* m) { return Eigen::Map<Eigen::Matrix3f const>(m->data).transpose(); }
    inline auto _to(XMat33 * m) { return Eigen::Map<Eigen::Matrix3f>(m->data).transpose(); }

    inline auto _to(XMat33 const& m) { return Eigen::Map<Eigen::Matrix3f const>(m.data).transpose(); }
    inline auto _to(XMat33 & m) { return Eigen::Map<Eigen::Matrix3f>(m.data).transpose(); }

    inline auto _to(XMat66 const& m) { return Eigen::Map<Eigen::Matrix<Float,6,6> const>(m.data).transpose(); }
    inline auto _to(XMat66 & m) { return Eigen::Map<Eigen::Matrix<Float,6,6>>(m.data).transpose(); }

    inline auto _to(XVec7 const* m) { return Eigen::Map<Eigen::Matrix<Float,7,1> const>(m->data); }
    inline auto _to(XVec7 * m) { return Eigen::Map<Eigen::Matrix<Float,7,1>>(m->data); }

    // auto _to(XMat26 const* m) { return Eigen::Map<Eigen::Matrix<Float,2,6> const>(m->data); }
    // auto _to(XMat26 * m) { return Eigen::Map<Eigen::Matrix<Float,2,6>>(m->data); }
    inline auto _to(XMat26 const* m) { return Eigen::Map<Eigen::Matrix<Float,6,2> const>(m->data).transpose(); }
    inline auto _to(XMat26 * m) { return Eigen::Map<Eigen::Matrix<Float,6,2>>(m->data).transpose(); }

    inline auto _to(XMat39 const& m) { return Eigen::Map<Eigen::Matrix<Float,9,3> const>(m.data).transpose(); }
    inline auto _to(XMat39 & m) { return Eigen::Map<Eigen::Matrix<Float,9,3>>(m.data).transpose(); }

    inline auto _to(XMat99 const& m) { return Eigen::Map<Eigen::Matrix<Float,9,9> const>(m.data).transpose(); }
    inline auto _to(XMat99 & m) { return Eigen::Map<Eigen::Matrix<Float,9,9>>(m.data).transpose(); }


  
#endif


