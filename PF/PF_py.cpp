#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PF.hpp"

namespace py = pybind11;

// Wrapper for ObsData to expose it to Python
void init_obs_data(py::module &m) {
    py::class_<ObsData>(m, "ObsData")
        .def(py::init<>())
        .def_readwrite("timestamp", &ObsData::timetamp) // Note: C++ struct has 'timetamp' typo, mapping to 'timestamp' in Python
        .def_readwrite("x", &ObsData::x)
        .def_readwrite("y", &ObsData::y)
        .def_readwrite("range", &ObsData::range)
        .def_readwrite("rngvalid", &ObsData::rngvalid)
        .def_readwrite("bearing", &ObsData::bearing)
        .def_readwrite("brgvalid", &ObsData::brgvalid)
        .def_readwrite("freq", &ObsData::freq)
        .def_readwrite("freqvalid", &ObsData::freqvalid);
}

// Wrapper for TargetState
void init_target_state(py::module &m) {
    py::class_<TargetState>(m, "TargetState")
        .def(py::init<>())
        .def_readwrite("x", &TargetState::x)
        .def_readwrite("y", &TargetState::y)
        .def_readwrite("vx", &TargetState::vx)
        .def_readwrite("vy", &TargetState::vy)
        .def("__repr__", [](const TargetState &ts) {
            return "<TargetState x=" + std::to_string(ts.x) + 
                   " y=" + std::to_string(ts.y) + 
                   " vx=" + std::to_string(ts.vx) + 
                   " vy=" + std::to_string(ts.vy) + ">";
        });
}

// Wrapper for Particle
void init_particle(py::module &m) {
    py::class_<Particle>(m, "Particle")
        .def(py::init<>())
        .def_readwrite("x", &Particle::x)
        .def_readwrite("y", &Particle::y)
        .def_readwrite("vx", &Particle::vx)
        .def_readwrite("vy", &Particle::vy)
        .def_readwrite("weight", &Particle::weight);
}

PYBIND11_MODULE(PF_py, m) {
    m.doc() = "Particle Filter module for TMA (Python bindings)";

    init_obs_data(m);
    init_target_state(m);
    init_particle(m);

    py::class_<ParticleFilter>(m, "ParticleFilter")
        .def(py::init<int>(), py::arg("num_particles"))
        .def("initialize", &ParticleFilter::initialize,
             py::arg("obs"),
             py::arg("range_min"), py::arg("range_max"),
             py::arg("speed_min"), py::arg("speed_max"),
             py::arg("course_min"), py::arg("course_max"),
             py::arg("bearing_std"))
        .def("predict", &ParticleFilter::predict,
             py::arg("dt"),
             py::arg("process_noise_pos"),
             py::arg("process_noise_vel"))
        .def("update", &ParticleFilter::update,
             py::arg("obs"),
             py::arg("bearing_std"))
        .def("resample", &ParticleFilter::resample)
        .def("getEstimate", &ParticleFilter::getEstimate)
        .def("getParticles", &ParticleFilter::getParticles)
        .def("isInitialized", &ParticleFilter::isInitialized);
}
