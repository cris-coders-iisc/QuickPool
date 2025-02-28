#include "circuit.h"

namespace common::utils {

Gate::Gate(GateType type, wire_t out) : type(type), out(out) {}
Gate::Gate(GateType type, wire_t out, int rider_id, int driver_id) : type(type), out(out), rider_id(rider_id), driver_id(driver_id) {};

FIn2Gate::FIn2Gate(GateType type, wire_t in1, wire_t in2, wire_t out)
    : Gate(type, out), in1{in1}, in2{in2} {}

FIn2Gate::FIn2Gate(GateType type, wire_t in1, wire_t in2, wire_t out, int rider_id, int driver_id)
    : Gate(type, out, rider_id, driver_id), in1{in1}, in2{in2} {}

SIMDGate::SIMDGate(GateType type, std::vector<wire_t> in1, std::vector<wire_t> in2, wire_t out)
    : Gate(type, out), in1(std::move(in1)), in2(std::move(in2)) {}

SIMDGate::SIMDGate(GateType type, std::vector<wire_t> in1, std::vector<wire_t> in2, wire_t out, int rider_id, int driver_id)
    : Gate(type, out, rider_id, driver_id), in1(std::move(in1)), in2(std::move(in2)) {}

std::ostream& operator<<(std::ostream& os, GateType type) {
  switch (type) {
    case kInp:
      os << "Input";
      break;

    case kAdd:
      os << "Addition";
      break;

    case kMul:
      os << "Multiplication";
      break;

    case kSub:
      os << "Subtraction";
      break;

    case kConstAdd:
      os << "Addition with constant";
      break;

    case kConstMul:
      os << "Multiplication with constant";
      break;

    case kDotprod:
      os << "Dotproduct";
      break;

    default:
      os << "Invalid";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const LevelOrderedCircuit& circ) {
  for (size_t i = 0; i < GateType::NumGates; ++i) {
    os << GateType(i) << ": " << circ.count[i] << "\n";
  }
  os << "Total: " << circ.num_gates << "\n";
  os << "Depth: " << circ.gates_by_level.size() << "\n";
  return os;
}
};  // namespace common::utils
