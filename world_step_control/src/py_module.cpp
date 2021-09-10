#include "world_step_control/py_module.h"

#include <world_step_control/ModuleExecutionResult.h>

namespace py = boost::python;

struct PyExecResult
        : public world_step_control::ModuleExecutionResult
{
	PyExecResult(uint32_t startSec, uint32_t startNsec, uint32_t execSec, uint32_t execNsec, const std::string &name)
	{
		this->PauseTime = ros::Duration(startSec, startNsec);
		this->ExecutionTime = ros::Duration(execSec, execNsec);
		this->ModuleName = name;
	}
};

struct PyGILStateWrapper
{
	PyGILStateWrapper()
	    : _state(PyGILState_Ensure())
	{}

	~PyGILStateWrapper()
	{
		PyGILState_Release(this->_state);
	}

	private:
	    PyGILState_STATE _state;
};

struct PyModuleWrapper
        : public StepModule,
          boost::python::wrapper<StepModule>
{
	PyModuleWrapper(const std::string &name)
	    : StepModule(name, 0, nullptr, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler)
	{}

	ModuleExecutionResult ExecuteStep(const ros::Time &time) override
	{
		PyGILStateWrapper gilWrap;
		return this->get_override("_PyExecuteStep")(time.sec, time.nsec);
	}
};

BOOST_PYTHON_MODULE(PyStepModule)
{
	py::class_<world_step_control::ModuleExecutionResult>("__ModuleExecutionResult", py::no_init);

	py::class_<PyExecResult, py::bases<world_step_control::ModuleExecutionResult> >("_PyExecResult",
	                                                                                py::init<uint32_t, uint32_t, uint32_t, uint32_t, std::string>());

	py::class_<PyModuleWrapper, boost::noncopyable>("_PyStepModule", py::init<std::string>())
	        .def("Run", &StepModule::Run)
	        .def("RunOnce", &StepModule::RunOnce)
	        .def("ParamExecutionTime", &StepModule::ParamExecutionTime);
	        //.def("_PyExecuteStep", py::pure_virtual(&StepModule::ExecuteStep));
}
