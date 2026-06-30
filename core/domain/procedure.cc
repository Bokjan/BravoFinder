#include "core/domain/procedure.h"

namespace bf {

bool TerminatesAtFix(PathTerminator t) {
  switch (t) {
    case PathTerminator::kTF:
    case PathTerminator::kIF:
    case PathTerminator::kDF:
    case PathTerminator::kCF:
      return true;
    default:
      return false;
  }
}

PathTerminator ParsePathTerminator(std::string_view token) {
  if (token == "TF") return PathTerminator::kTF;
  if (token == "IF") return PathTerminator::kIF;
  if (token == "DF") return PathTerminator::kDF;
  if (token == "CF") return PathTerminator::kCF;
  if (token == "CA") return PathTerminator::kCA;
  if (token == "FM") return PathTerminator::kFM;
  if (token == "VA") return PathTerminator::kVA;
  if (token == "VM") return PathTerminator::kVM;
  if (token == "VI") return PathTerminator::kVI;
  if (token == "VR") return PathTerminator::kVR;
  if (token == "RF") return PathTerminator::kRF;
  if (token == "VD") return PathTerminator::kVD;
  if (token == "HM") return PathTerminator::kHM;
  if (token == "HF") return PathTerminator::kHF;
  return PathTerminator::kUnknown;
}

std::string PathTerminatorName(PathTerminator t) {
  switch (t) {
    case PathTerminator::kTF:
      return "TF";
    case PathTerminator::kIF:
      return "IF";
    case PathTerminator::kDF:
      return "DF";
    case PathTerminator::kCF:
      return "CF";
    case PathTerminator::kCA:
      return "CA";
    case PathTerminator::kFM:
      return "FM";
    case PathTerminator::kVA:
      return "VA";
    case PathTerminator::kVM:
      return "VM";
    case PathTerminator::kVI:
      return "VI";
    case PathTerminator::kVR:
      return "VR";
    case PathTerminator::kRF:
      return "RF";
    case PathTerminator::kVD:
      return "VD";
    case PathTerminator::kHM:
      return "HM";
    case PathTerminator::kHF:
      return "HF";
    case PathTerminator::kUnknown:
      return "??";
  }
  return "??";
}

}  // namespace bf
