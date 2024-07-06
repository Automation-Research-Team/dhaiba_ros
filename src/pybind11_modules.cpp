/*!
*  \file	pybind11_modules.cpp
*  \author	Toshio UESHIBA
*  \brief	Define python3 bindings for note_publisher and note_subscriber
*/
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include "dhaiba_note.h"

namespace py = pybind11;

PYBIND11_MODULE(dhaiba_ros, m)
{
    m.doc() = R"pbdoc(Pybind11 dhaiba plugin)pbdoc";

    using namespace dhaiba_ros;

    py::class_<NotePublisher>(m, "note_publisher")
        .def(py::init<const std::string&, const std::string&>())
        .def("write", &NotePublisher::write)
        .def("my_test", &NotePublisher::my_test);

    py::class_<NoteSubscriber>(m, "note_subscriber")
        .def(py::init<const std::string&, const std::string&, py::function>())
        .def("my_test", &NoteSubscriber::my_test);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
