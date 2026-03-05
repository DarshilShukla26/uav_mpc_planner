#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <vector>

namespace py = pybind11;

extern "C" {
int mpc_solver_func_work(int *sz_arg, int *sz_res, int *sz_iw, int *sz_w);
int mpc_solver_func(const double **arg, double **res, int *iw, double *w,
                    int mem);
}

class MPCSolverWrapper {
private:
  std::vector<int> iw;
  std::vector<double> w;
  std::vector<const double *> arg;
  std::vector<double *> res;

public:
  MPCSolverWrapper() {
    int sz_arg = 0, sz_res = 0, sz_iw = 0, sz_w = 0;
    mpc_solver_func_work(&sz_arg, &sz_res, &sz_iw, &sz_w);

    iw.resize(sz_iw);
    w.resize(sz_w);
    arg.resize(sz_arg);
    res.resize(sz_res);
  }

  py::tuple solve(py::array_t<double> x0, py::array_t<double> X_ref,
                  py::array_t<double> obs_params, py::array_t<double> f_wind) {

    arg[0] = static_cast<const double *>(x0.data());
    arg[1] = static_cast<const double *>(X_ref.data());
    arg[2] = static_cast<const double *>(obs_params.data());
    arg[3] = static_cast<const double *>(f_wind.data());

    py::array_t<double> U_opt(80);
    py::array_t<double> X_opt(252);

    res[0] = U_opt.mutable_data();
    res[1] = X_opt.mutable_data();

    int status =
        mpc_solver_func(arg.data(), res.data(), iw.data(), w.data(), 0);

    return py::make_tuple(U_opt, X_opt, status);
  }
};

PYBIND11_MODULE(mpc_solver_py, m) {
  py::class_<MPCSolverWrapper>(m, "MPCSolverWrapper")
      .def(py::init<>())
      .def("solve", &MPCSolverWrapper::solve);
}
