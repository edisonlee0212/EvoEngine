#pragma once

using namespace evo_engine;

namespace digital_agriculture
{
	class SorghumDescriptor : public IAsset
	{
	public:
		float m_internodeLength;
		float m_branchingAngle;
	};
}