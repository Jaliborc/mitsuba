#include <mitsuba/render/scene.h>
#include <mitsuba/core/shvector.h>
#include <math.h>
#include <stdio.h>

MTS_NAMESPACE_BEGIN

class SHMap : public SamplingIntegrator {
public:
	SHMap(const Properties &props) : SamplingIntegrator(props) {
		double harmonic = (double) props.getInteger("harmonic", 0);
		int l = (int) ceil(pow(harmonic + 1, .5) - 1);
		int m = harmonic - l*(l+1);

		data = SHVector(l + 1);
		data(l,m) = props.getFloat("scale", 1);
	}

	Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &record) const {
		float amount = data.eval(ray.d);
		if (amount > 0.0f)
			return Spectrum(amount);
		else
			return Spectrum(0.0f);
	}

private:
	SHVector data;

MTS_DECLARE_CLASS()
};

MTS_IMPLEMENT_CLASS(SHMap, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(SHMap, "Generates spherical harmonic maps");
MTS_NAMESPACE_END