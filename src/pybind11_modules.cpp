#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include "dhaiba.h"

namespace py = pybind11;

PYBIND11_MODULE(LIBRARY_NAME, m) {
    m.doc() = R"pbdoc(
        Pybind11 dhaiba plugin
    )pbdoc";

    using namespace dhaiba_ros;

    py::class_<note_publisher>(m, "note_publisher")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&>())
        .def("write", &note_publisher::write)
        .def("my_test", &note_publisher::my_test)
        ;

    py::class_<note_subscriber>(m, "note_subscriber")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&, py::function>())
        .def("my_test", &note_subscriber::my_test)
        ;

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
