#if !defined(__LAYERED_INTEGRATOR_H)
#define __LAYERED_INTEGRATOR_H
#include <mitsuba/render/scene.h>

MTS_NAMESPACE_BEGIN

class LayeredIntegrator : public SamplingIntegrator {
public:
	void addChild(const std::string &name, ConfigurableObject *child) {
		const Class *cClass = child->getClass();

		if (cClass->derivesFrom(MTS_CLASS(Integrator))) {
			if (!cClass->derivesFrom(MTS_CLASS(SamplingIntegrator)))
				Log(EError, "The sub-integrator must be derived from the SamplingIntegrator class.");
			m_subIntegrator = static_cast<SamplingIntegrator*>(child);
			m_subIntegrator->setParent(this);
		} else {
			Integrator::addChild(name, child);
		}
	}

	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID) {
		if (m_subIntegrator == NULL)
			Log(EError, "No sub-integrator was specified!");

		return m_subIntegrator->preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
	}

	void postprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID) {
		m_subIntegrator->postprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
	}

	void configureSampler(const Scene *scene, Sampler *sampler) {
		m_subIntegrator->configureSampler(scene, sampler);
	}

	void bindUsedResources(ParallelProcess *proc) const {
		m_subIntegrator->bindUsedResources(proc);
	}

	void wakeup(ConfigurableObject *parent, std::map<std::string, SerializableObject *> &params) {
		m_subIntegrator->wakeup(this, params);
	}

	void cancel() {
		m_subIntegrator->cancel();
	}

	Spectrum Li(const RayDifferential &ray, RadianceQueryRecord &record) const {
		return m_subIntegrator->Li(ray, record);
	}

	Spectrum E(const Scene *scene, const Intersection &its, const Medium *medium, Sampler *sampler, int nSamples, bool includeIndirect) const {
		return m_subIntegrator->E(scene, its, medium, sampler, nSamples, includeIndirect);
	}

	const Integrator *getSubIntegrator() const {
		return m_subIntegrator.get();
	}

MTS_DECLARE_CLASS()
protected:
	LayeredIntegrator(const Properties &props) : SamplingIntegrator(props) {};
	LayeredIntegrator(Stream *stream, InstanceManager *manager) :SamplingIntegrator(stream, manager) {};
	ref<SamplingIntegrator> m_subIntegrator;
};

MTS_IMPLEMENT_CLASS(LayeredIntegrator, true, SamplingIntegrator)
MTS_NAMESPACE_END

#endif