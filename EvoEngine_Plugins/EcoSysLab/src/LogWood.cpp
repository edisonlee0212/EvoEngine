#include "LogWood.hpp"

using namespace eco_sys_lab;

float LogWoodIntersection::GetCenterDistance(const float angle) const {
  assert(m_boundary.size() == 360);
  const int startIndex = static_cast<int>(angle) % 360;
  const float a = angle - static_cast<int>(angle);
  const int endIndex = (startIndex + 1) % 360;
  return glm::mix(m_boundary[startIndex].m_centerDistance, m_boundary[endIndex].m_centerDistance, a);
}

glm::vec2 LogWoodIntersection::GetBoundaryPoint(const float angle) const {
  return m_center + glm::vec2(glm::cos(glm::radians(angle)), glm::sin(glm::radians(angle))) * GetCenterDistance(angle);
}

float LogWoodIntersection::GetDefectStatus(const float angle) const {
  assert(m_boundary.size() == 360);
  const int startIndex = static_cast<int>(angle) % 360;
  const float a = angle - static_cast<int>(angle);
  const int endIndex = (startIndex + 1) % 360;
  return glm::mix(m_boundary[startIndex].m_defectStatus, m_boundary[endIndex].m_defectStatus, a);
}

glm::vec4 LogWoodIntersection::GetColor(const float angle) const {
  assert(m_boundary.size() == 360);
  const int startIndex = static_cast<int>(angle) % 360;
  const float a = angle - static_cast<int>(angle);
  const int endIndex = (startIndex + 1) % 360;
  return glm::mix(m_boundary[startIndex].m_color, m_boundary[endIndex].m_color, a);
}

float LogWoodIntersection::GetAverageDistance() const {
  float retVal = 0.0f;
  for (const auto& i : m_boundary)
    retVal += i.m_centerDistance;
  return retVal / m_boundary.size();
}

float LogWoodIntersection::GetMaxDistance() const {
  float retVal = FLT_MIN;
  for (const auto& i : m_boundary) {
    retVal = glm::max(retVal, i.m_centerDistance);
  }
  return retVal;
}

float LogWoodIntersection::GetMinDistance() const {
  float retVal = FLT_MAX;
  for (const auto& i : m_boundary) {
    retVal = glm::min(retVal, i.m_centerDistance);
  }
  return retVal;
}

glm::vec2 LogWood::GetSurfacePoint(const float height, const float angle) const {
  const float heightStep = m_length / m_intersections.size();
  const int index = static_cast<int>(height / heightStep);
  const float a = (height - heightStep * index) / heightStep;
  const auto actualAngle = glm::mod(angle, 360.0f);
  if (index < m_intersections.size() - 1) {
    return glm::mix(m_intersections.at(index).GetBoundaryPoint(actualAngle),
                    m_intersections.at(index + 1).GetBoundaryPoint(actualAngle), a);
  }
  return m_intersections.back().GetBoundaryPoint(actualAngle);
}

float LogWood::GetCenterDistance(const float height, const float angle) const {
  const float heightStep = m_length / m_intersections.size();
  const int index = height / heightStep;
  const float a = (height - heightStep * index) / heightStep;
  const auto actualAngle = glm::mod(angle, 360.0f);
  if (index < m_intersections.size() - 1) {
    return glm::mix(m_intersections.at(index).GetCenterDistance(actualAngle),
                    m_intersections.at(index + 1).GetCenterDistance(actualAngle), a);
  }
  return m_intersections.back().GetCenterDistance(actualAngle);
}

float LogWood::GetDefectStatus(const float height, const float angle) const {
  const float heightStep = m_length / m_intersections.size();
  const int index = height / heightStep;
  const float a = (height - heightStep * index) / heightStep;
  const auto actualAngle = glm::mod(angle, 360.0f);
  if (index < m_intersections.size() - 1) {
    return glm::mix(m_intersections.at(index).GetDefectStatus(actualAngle),
                    m_intersections.at(index + 1).GetDefectStatus(actualAngle), a);
  }
  return m_intersections.back().GetDefectStatus(actualAngle);
}

glm::vec4 LogWood::GetColor(const float height, const float angle) const {
  const float heightStep = m_length / m_intersections.size();
  const int index = height / heightStep;
  const float a = (height - heightStep * index) / heightStep;
  const auto actualAngle = glm::mod(angle, 360.0f);
  if (index < m_intersections.size() - 1) {
    return glm::mix(m_intersections.at(index).GetColor(actualAngle),
                    m_intersections.at(index + 1).GetColor(actualAngle), a);
  }
  return m_intersections.back().GetColor(actualAngle);
}

LogWoodIntersectionBoundaryPoint& LogWood::GetBoundaryPoint(const float height, const float angle) {
  const float heightStep = m_length / m_intersections.size();
  const int index = height / heightStep;
  const float a = (height - heightStep * index) / heightStep;
  const auto actualAngle = glm::mod(angle, 360.0f);
  if (index < m_intersections.size() - 1) {
    if (a < 0.5f) {
      return m_intersections.at(index).m_boundary.at(actualAngle);
    }
    return m_intersections.at(index + 1).m_boundary.at(actualAngle);
  }
  return m_intersections.back().m_boundary.at(actualAngle);
}

void LogWood::Rotate(int degrees) {
  while (degrees < 0.)
    degrees += 360;
  degrees = degrees % 360;
  if (degrees == 0)
    return;
  for (auto& intersection : m_intersections) {
    intersection.m_center = glm::rotate(intersection.m_center, glm::radians(static_cast<float>(degrees)));
    const std::vector<LogWoodIntersectionBoundaryPoint> last = {intersection.m_boundary.end() - degrees,
                                                                intersection.m_boundary.end()};
    std::copy(intersection.m_boundary.begin(), intersection.m_boundary.end() - degrees,
              intersection.m_boundary.begin() + degrees);
    std::copy(last.begin(), last.end(), intersection.m_boundary.begin());
  }
}

float LogWood::GetAverageDistance(const float height) const {
  const float heightStep = m_length / m_intersections.size();
  const int index = height / heightStep;
  const float a = (height - heightStep * index) / heightStep;
  if (index < m_intersections.size() - 1) {
    if (a < 0.5f) {
      return m_intersections.at(index).GetAverageDistance();
    }
    return m_intersections.at(index + 1).GetAverageDistance();
  }
  return m_intersections.back().GetAverageDistance();
}

float LogWood::GetAverageDistance() const {
  float sumDistance = 0.0f;
  for (const auto& intersection : m_intersections) {
    for (const auto& point : intersection.m_boundary)
      sumDistance += point.m_centerDistance;
  }
  return sumDistance / m_intersections.size() / 360.0f;
}

float LogWood::GetMaxAverageDistance() const {
  float retVal = FLT_MIN;
  for (const auto& i : m_intersections) {
    retVal = glm::max(i.GetAverageDistance(), retVal);
  }
  return retVal;
}

float LogWood::GetMinAverageDistance() const {
  float retVal = FLT_MAX;
  for (const auto& i : m_intersections) {
    retVal = glm::min(i.GetAverageDistance(), retVal);
  }
  return retVal;
}

float LogWood::GetMaxDistance() const {
  float retVal = FLT_MIN;
  for (const auto& i : m_intersections) {
    retVal = glm::max(i.GetMaxDistance(), retVal);
  }
  return retVal;
}

float LogWood::GetMinDistance() const {
  float retVal = FLT_MAX;
  for (const auto& i : m_intersections) {
    retVal = glm::min(i.GetMinDistance(), retVal);
  }
  return retVal;
}

void LogWood::MarkDefectRegion(const float height, const float angle, const float heightRange, const float angleRange) {
  const float heightStep = m_length / m_intersections.size();
  const int heightStepSize = heightRange / heightStep;
  for (int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++) {
    const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
    const auto sinAngle = glm::asin(sinVal);
    const float maxAngleRange = glm::cos(sinAngle);
    for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++) {
      const auto actualYIndex = yIndex + height / heightStep;
      if (actualYIndex < 0 || actualYIndex >= m_intersections.size())
        continue;
      if (glm::abs(static_cast<float>(xIndex)) / angleRange > maxAngleRange)
        continue;
      const auto actualXIndex = static_cast<int>(360 + xIndex + angle) % 360;
      m_intersections.at(actualYIndex).m_boundary.at(actualXIndex).m_defectStatus = 1.0f;
    }
  }
}

void LogWood::EraseDefectRegion(const float height, const float angle, const float heightRange,
                                const float angleRange) {
  const float heightStep = m_length / m_intersections.size();
  const int heightStepSize = heightRange / heightStep;
  for (int yIndex = -heightStepSize; yIndex <= heightStepSize; yIndex++) {
    const auto sinVal = glm::abs(static_cast<float>(yIndex)) / heightStepSize;
    const auto sinAngle = glm::asin(sinVal);
    const float maxAngleRange = glm::cos(sinAngle);
    for (int xIndex = -angleRange; xIndex <= angleRange; xIndex++) {
      const auto actualYIndex = yIndex + height / heightStep;
      if (actualYIndex < 0 || actualYIndex >= m_intersections.size())
        continue;
      if (glm::abs(static_cast<float>(xIndex)) / angleRange > maxAngleRange)
        continue;
      const auto actualXIndex = static_cast<int>(xIndex + angle) % 360;
      m_intersections.at(actualYIndex).m_boundary.at(actualXIndex).m_defectStatus = 0.0f;
    }
  }
}

void LogWood::ClearDefects() {
  for (auto& intersection : m_intersections) {
    for (auto& boundaryPoint : intersection.m_boundary) {
      boundaryPoint.m_defectStatus = 0.0f;
      boundaryPoint.m_color = glm::vec4(212.f / 255, 175.f / 255, 55.f / 255, 1);
    }
  }
}

bool LogWood::RayCastSelection(const glm::mat4& transform, const float pointDistanceThreshold, const Ray& ray,
                               float& height, float& angle) const {
  float minDistance = FLT_MAX;
  bool found = false;
  const float heightStep = m_length / m_intersections.size();
  for (int yIndex = 0; yIndex < m_intersections.size(); yIndex++) {
    const auto testHeight = yIndex * heightStep;
    for (int xIndex = 0; xIndex < 360; xIndex++) {
      const auto surfacePoint = GetSurfacePoint(testHeight, xIndex);
      const glm::vec3 position = (transform * glm::translate(glm::vec3(surfacePoint.x, testHeight, surfacePoint.y)))[3];
      glm::vec3 closestPoint = Ray::ClosestPointOnLine(position, ray.start, ray.start + ray.direction * 10000.0f);
      const float pointDistance = glm::distance(closestPoint, position);
      const float distanceToStart = glm::distance(ray.start, position);
      if (distanceToStart < minDistance && pointDistance < pointDistanceThreshold) {
        minDistance = distanceToStart;
        height = testHeight;
        angle = xIndex;
        found = true;
      }
    }
  }
  return found;
}

std::vector<LogGradeFaceCutting> CalculateCuttings(const float trim, const std::vector<bool>& defectMarks,
                                                   const float heightStep, const float minDistance,
                                                   const float startHeight) {
  int lastDefectIndex = 0.0f;
  std::vector<LogGradeFaceCutting> cuttings{};
  for (int intersectionIndex = 0; intersectionIndex < defectMarks.size(); intersectionIndex++) {
    if (defectMarks[intersectionIndex] || intersectionIndex == defectMarks.size() - 1) {
      if (heightStep * (intersectionIndex - lastDefectIndex) >= minDistance + trim + trim) {
        LogGradeFaceCutting cutting;
        cutting.m_startInMeters = heightStep * lastDefectIndex + startHeight + trim;
        cutting.m_endInMeters = heightStep * intersectionIndex + startHeight - trim;
        cuttings.emplace_back(cutting);
      }
      lastDefectIndex = intersectionIndex;
    }
  }
  return cuttings;
}

bool TestCutting(const std::vector<LogGradeFaceCutting>& cuttings, const int maxNumber, const float minProportion,
                 const float logLength, float& minCuttingLength, float& proportion) {
  const auto cuttingNumber = cuttings.size();
  if (cuttingNumber == 0)
    return false;
  float cuttingSumLength = 0.0f;
  minCuttingLength = FLT_MAX;
  for (const auto& cutting : cuttings) {
    const float length = cutting.m_endInMeters - cutting.m_startInMeters;
    cuttingSumLength += length;
    minCuttingLength = glm::min(minCuttingLength, length);
  }
  proportion = cuttingSumLength / logLength;
  if (cuttingNumber <= maxNumber && proportion >= minProportion) {
    return true;
  }
  return false;
}

float LogWood::InchesToMeters(const float inches) {
  return inches * 0.0254f;
}

float LogWood::FeetToMeter(const float feet) {
  return feet * 0.3048f;
}

float LogWood::MetersToInches(const float meters) {
  return meters * 39.3701f;
}

float LogWood::MetersToFeet(const float meters) {
  return meters * 3.28084f;
}

void LogWood::CalculateGradingData(std::vector<LogGrading>& logGrading) const {
  std::vector<LogGrading> results{};
  const float intersectionLength = m_length / m_intersections.size();
  const int gradingSectionIntersectionCount =
      glm::min(glm::ceil(FeetToMeter(12) / intersectionLength), static_cast<float>(m_intersections.size()) - 1);

  float cuttingTrim = InchesToMeters(3);
  for (int startIntersectionIndex = 0;
       startIntersectionIndex < m_intersections.size() - gradingSectionIntersectionCount; startIntersectionIndex++) {
    for (int angleOffset = 0; angleOffset < 90; angleOffset++) {
      results.emplace_back();
      auto& tempLogGrading = results.back();
      tempLogGrading.m_angleOffset = angleOffset;
      // TODO: Calculate Scaling Diameter correctly.
      tempLogGrading.m_scalingDiameterInMeters = GetMinAverageDistance() * 2.f;
      tempLogGrading.m_startHeightInMeters = startIntersectionIndex * intersectionLength;
      tempLogGrading.m_startIntersectionIndex = startIntersectionIndex;
      tempLogGrading.m_lengthWithoutTrimInMeters = gradingSectionIntersectionCount * intersectionLength;
      const int d = static_cast<int>(glm::round(MetersToInches(tempLogGrading.m_scalingDiameterInMeters)));
      const int l = static_cast<int>(glm::round(MetersToFeet(tempLogGrading.m_lengthWithoutTrimInMeters)));
      tempLogGrading.m_doyleRuleScale = (d - 4.f) * (d - 4.f) * l / 16.f;
      tempLogGrading.m_scribnerRuleScale = (0.79f * d * d - d * 2.f - 4.f) * l / 16.f;
      tempLogGrading.m_internationalRuleScale =
          static_cast<float>(0.04976191 * l * d * d + 0.006220239 * l * l * d - 0.1854762 * l * d +
                             0.0002591767 * l * l * l - 0.01159226 * l * l + 0.04222222 * l);

      if (l <= 10) {
        tempLogGrading.m_sweepDeduction = glm::max(0.0f, (m_sweepInInches - 1.f) / d);
      } else if (l <= 13) {
        tempLogGrading.m_sweepDeduction = glm::max(0.0f, (m_sweepInInches - 1.5f) / d);
      } else {
        tempLogGrading.m_sweepDeduction = glm::max(0.0f, (m_sweepInInches - 2.f) / d);
      }
      tempLogGrading.m_crookDeduction = m_crookCInInches / d * m_crookCLInFeet / l;

      for (int faceIndex = 0; faceIndex < 4; faceIndex++) {
        auto& face = tempLogGrading.m_faces[faceIndex];
        face.m_startAngle = (angleOffset + faceIndex * 90) % 360;
        face.m_endAngle = (angleOffset + faceIndex * 90 + 90) % 360;
        std::vector<bool> defectMarks;
        defectMarks.resize(gradingSectionIntersectionCount);
        for (int intersectionIndex = 0; intersectionIndex < gradingSectionIntersectionCount; intersectionIndex++) {
          const auto& intersection = m_intersections[intersectionIndex + startIntersectionIndex];
          for (int angle = 0; angle < 90; angle++) {
            const int actualAngle = (face.m_startAngle + angle) % 360;
            if (intersection.m_boundary[actualAngle].m_defectStatus != 0.0f) {
              defectMarks[intersectionIndex] = true;
              break;
            }
          }
        }

        bool f1Possible = true;

        if (!m_soundDefect) {
          if (tempLogGrading.m_crookDeduction > 0.15f || tempLogGrading.m_sweepDeduction > 0.15f) {
            f1Possible = false;
          }
        } else {
          if (tempLogGrading.m_crookDeduction > 0.1f || tempLogGrading.m_sweepDeduction > 0.1f) {
            f1Possible = false;
          }
        }

        bool f2Possible = true;
        if (!f1Possible) {
          if (!m_soundDefect) {
            if (tempLogGrading.m_crookDeduction > 0.3f || tempLogGrading.m_sweepDeduction > 0.3f) {
              f2Possible = false;
            }
          } else {
            if (tempLogGrading.m_crookDeduction > 0.2f || tempLogGrading.m_sweepDeduction > 0.2f) {
              f2Possible = false;
            }
          }
        }
        bool f3Possible = true;
        if (!f2Possible) {
          if (!m_soundDefect) {
            if (tempLogGrading.m_crookDeduction > 0.5f || tempLogGrading.m_sweepDeduction > 0.5f) {
              f3Possible = false;
            }
          } else {
            if (tempLogGrading.m_crookDeduction > 0.35f || tempLogGrading.m_sweepDeduction > 0.35f) {
              f3Possible = false;
            }
          }
        }
        bool succeed = false;
        if (f1Possible && m_butt && tempLogGrading.m_scalingDiameterInMeters >= InchesToMeters(13) &&
            tempLogGrading.m_scalingDiameterInMeters <= InchesToMeters(16) &&
            tempLogGrading.m_lengthWithoutTrimInMeters >= FeetToMeter(10)) {
          // F1: Butt, Scaling diameter 13-15(16), Length 10+
          const auto cuttings7 = CalculateCuttings(cuttingTrim, defectMarks, intersectionLength, FeetToMeter(7),
                                                   tempLogGrading.m_startHeightInMeters);
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings7, 2, 5.0f / 6.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);

          if (succeed) {
            face.m_faceGrade = 1;
            face.m_clearCuttings = cuttings7;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        if (f1Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(16) &&
            tempLogGrading.m_scalingDiameterInMeters <= InchesToMeters(20) &&
            tempLogGrading.m_lengthWithoutTrimInMeters >= FeetToMeter(10)) {
          // F1: Butt & uppers, Scaling diameter 16-19(20), Length 10+
          const auto cuttings5 = CalculateCuttings(cuttingTrim, defectMarks, intersectionLength, FeetToMeter(5),
                                                   tempLogGrading.m_startHeightInMeters);
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings5, 2, 5.0f / 6.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);

          if (succeed) {
            face.m_faceGrade = 2;
            face.m_clearCuttings = cuttings5;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        const auto cuttings3 = CalculateCuttings(cuttingTrim, defectMarks, intersectionLength, FeetToMeter(3),
                                                 tempLogGrading.m_startHeightInMeters);
        if (f1Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(20) &&
            tempLogGrading.m_lengthWithoutTrimInMeters >= FeetToMeter(10)) {
          // F1: Butt & uppers, Scaling diameter 20+, Length 10+
          // const auto cuttings3 = CalculateCuttings(defectMarks, heightStep, FeetToMeter(3));
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 5.0f / 6.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);

          if (succeed) {
            face.m_faceGrade = 3;
            face.m_clearCuttings = cuttings3;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }

        if (f2Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(11) &&
            tempLogGrading.m_lengthWithoutTrimInMeters >= FeetToMeter(10)) {
          // F2: Butt & uppers, Scaling diameter 11+, Length 10+
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);

          if (succeed) {
            face.m_faceGrade = 4;
            face.m_clearCuttings = cuttings3;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        if (f2Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(12) &&
            tempLogGrading.m_lengthWithoutTrimInMeters > FeetToMeter(8) &&
            tempLogGrading.m_lengthWithoutTrimInMeters <= FeetToMeter(10)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 8-9(10)
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 3.0f / 4.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);
          if (succeed) {
            if (!m_soundDefect) {
              if (tempLogGrading.m_crookDeduction > 0.3f || tempLogGrading.m_sweepDeduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (tempLogGrading.m_crookDeduction > 0.2f || tempLogGrading.m_sweepDeduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_faceGrade = 5;
            face.m_clearCuttings = cuttings3;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        if (f2Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(12) &&
            tempLogGrading.m_lengthWithoutTrimInMeters > FeetToMeter(10) &&
            tempLogGrading.m_lengthWithoutTrimInMeters <= FeetToMeter(12)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 10-11(12)
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 2, 2.0f / 3.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);
          if (succeed) {
            if (!m_soundDefect) {
              if (tempLogGrading.m_crookDeduction > 0.3f || tempLogGrading.m_sweepDeduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (tempLogGrading.m_crookDeduction > 0.2f || tempLogGrading.m_sweepDeduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_faceGrade = 6;
            face.m_clearCuttings = cuttings3;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        if (f2Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(12) &&
            tempLogGrading.m_lengthWithoutTrimInMeters > FeetToMeter(12)) {
          // F2: Butt & uppers, Scaling diameter 12+, Length 12+
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings3, 3, 2.0f / 3.0f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);
          if (succeed) {
            if (!m_soundDefect) {
              if (tempLogGrading.m_crookDeduction > 0.3f || tempLogGrading.m_sweepDeduction > 0.3f) {
                succeed = false;
              }
            } else {
              if (tempLogGrading.m_crookDeduction > 0.2f || tempLogGrading.m_sweepDeduction > 0.2f) {
                succeed = false;
              }
            }
          }
          if (succeed) {
            face.m_faceGrade = 7;
            face.m_clearCuttings = cuttings3;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        const auto cuttings2 = CalculateCuttings(cuttingTrim, defectMarks, intersectionLength, FeetToMeter(2),
                                                 tempLogGrading.m_startHeightInMeters);
        if (f3Possible && !succeed && tempLogGrading.m_scalingDiameterInMeters > InchesToMeters(8) &&
            tempLogGrading.m_lengthWithoutTrimInMeters > FeetToMeter(8)) {
          // F3: Butt & uppers, Scaling diameter 8+, Length 8+
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings2, 999, 1.0f / 2.0f, tempLogGrading.m_lengthWithoutTrimInMeters,
                                minCuttingLength, proportion);

          if (succeed) {
            face.m_faceGrade = 8;
            face.m_clearCuttings = cuttings2;
            face.m_clearCuttingMinLengthInMeters = minCuttingLength;
            face.m_clearCuttingMinProportion = proportion;
          }
        }
        if (!succeed) {
          float minCuttingLength = 0.0f;
          float proportion = 0.0f;
          succeed = TestCutting(cuttings2, 999, 0.f, tempLogGrading.m_lengthWithoutTrimInMeters, minCuttingLength,
                                proportion);
          face.m_faceGrade = 9;
          face.m_clearCuttings = cuttings2;
          face.m_clearCuttingMinLengthInMeters = minCuttingLength;
          face.m_clearCuttingMinProportion = proportion;
        }
      }
      int worstGradeIndex = -1;
      int worstGrade = 0;
      for (int i = 0; i < 4; i++) {
        if (tempLogGrading.m_faces[i].m_faceGrade > worstGrade) {
          worstGradeIndex = i;
          worstGrade = tempLogGrading.m_faces[i].m_faceGrade;
        }
      }
      int secondWorstGradeIndex = -1;
      worstGrade = 0;
      for (int i = 0; i < 4; i++) {
        if (i != worstGradeIndex && tempLogGrading.m_faces[i].m_faceGrade > worstGrade) {
          secondWorstGradeIndex = i;
          worstGrade = tempLogGrading.m_faces[i].m_faceGrade;
        }
      }
      tempLogGrading.m_gradeDetermineFaceIndex = secondWorstGradeIndex;
      tempLogGrading.m_grade = worstGrade;
    }
  }
  int bestGrade = INT_MAX;
  for (const auto& result : results) {
    if (result.m_grade < bestGrade) {
      bestGrade = result.m_grade;
      logGrading.clear();
      logGrading.emplace_back(result);
    } else if (result.m_grade == bestGrade) {
      logGrading.emplace_back(result);
    }
  }
}

void LogWood::ColorBasedOnGrading(const LogGrading& logGradingData) {
  const float intersectionLength = m_length / m_intersections.size();
  const int gradingSectionIntersectionCount = glm::ceil(FeetToMeter(12) / intersectionLength);
  for (const auto& face : logGradingData.m_faces) {
    for (int intersectionIndex = 0; intersectionIndex < m_intersections.size(); intersectionIndex++) {
      auto& intersection = m_intersections[intersectionIndex];
      for (int angle = 0; angle < 90; angle++) {
        const int actualAngle = (face.m_startAngle + angle) % 360;
        auto& point = intersection.m_boundary[actualAngle];

        if (point.m_defectStatus != 0.0f) {
          point.m_color = glm::vec4(1, 0, 0, 1);
          continue;
        }
        const float height = intersectionLength * intersectionIndex;
        bool isCutting = false;
        for (const auto& cutting : face.m_clearCuttings) {
          if (height >= cutting.m_startInMeters && height <= cutting.m_endInMeters) {
            if (face.m_faceGrade <= 3) {
              point.m_color = glm::vec4(212.f / 255, 175.f / 255, 55.f / 255, 1);
            } else if (face.m_faceGrade <= 7) {
              point.m_color = glm::vec4(170.f / 255, 169.f / 255, 173.f / 255, 1);
            } else if (face.m_faceGrade <= 8) {
              point.m_color = glm::vec4(131.f / 255, 105.f / 255, 83.f / 255, 1);
            } else {
              point.m_color = glm::vec4(0.1f, 0.1f, 0.1f, 1);
            }

            isCutting = true;
            break;
          }
        }
        if (!isCutting) {
          if (intersectionIndex < logGradingData.m_startIntersectionIndex ||
              intersectionIndex > logGradingData.m_startIntersectionIndex + gradingSectionIntersectionCount) {
            point.m_color = glm::vec4(0, 0, 0, 1);
          } else {
            point.m_color = glm::vec4(0.2f, 0.0f, 0.0f, 1);
          }
        }
      }
    }
  }
}
