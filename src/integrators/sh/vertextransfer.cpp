#include <sstream>
#include <mitsuba/render/scene.h>
#include <mitsuba/core/fstream.h>
#include "layeredintegrator.h"
#define NUM_WORKERS 8
using namespace std;

MTS_NAMESPACE_BEGIN

class VertexTransfer;
class TransferWorker : public Thread {
public:
	TransferWorker(VertexTransfer*, int);
	void run();

	ref<VertexTransfer> parent;
	int index;
};

struct Target {
	inline Target(ref<Shape> _shape, Point& _point, Vector& _normal, Point2& _uv) :
		shape(_shape), point(_point), normal(_normal), uv(_uv) {}

	ref<Shape> shape;
	Point point;
	Vector normal;
	Point2 uv;
};

class VertexTransfer : public LayeredIntegrator {
public:
	VertexTransfer(const Properties &props) : LayeredIntegrator(props) {
		int numBands = props.getInteger("numBands", 4);
		mapsRoot = props.getString("mapsRoot", "shmaps");
		numHarmonics = numBands * numBands;
	}

	bool render(Scene* _scene, RenderQueue* queue, const RenderJob* job, int sceneID, int sensorID, int samplerID) {
		// Initialize
		scene = _scene;
		medium = scene->getSensor()->getMedium();
		targets.clear();

		// Generate target list
		ref_vector<Shape> shapes = scene->getShapes();

		for (size_t s = 0; s < shapes.size(); ++s) {
			ref<Shape> shape = shapes[s].get();
			ref<TriMesh> mesh = shape->createTriMesh();
			Point2* coords = mesh->getVertexTexcoords();
			Normal* normals = mesh->getVertexNormals();
			Point* points = mesh->getVertexPositions();
			Point* vertices = mesh->getVertices();

			for (size_t i = 0; i < mesh->numVertices(); ++i) {
				Vector normal = Vector(0.0f, 0.0f, 0.0f);
				size_t index;

				for (size_t k = 0; k < mesh->getVertexCount(); ++k)
					if (points[k] == vertices[i]) {
						normal += Vector(normals[k]);
						index = k;
					}

				if (normal.isZero())
					normal = normals[index];
				else
					normal = normalize(normal);

				targets.push_back(Target(shape, vertices[i], normal, coords[index]));

				if (normal.isZero())
					Log(EError, "Point %s giving null normal.", vertices[i].toString().c_str());
				else
					Log(EDebug, "Found %dth point: %s -> %s", i, vertices[i].toString().c_str(), normal.toString().c_str());
			}
		}

		Log(EInfo, "Generated vertices list, found %d points.", targets.size());

		// Compute coefficients
		const Emitter* emitter = scene->getEnvironmentEmitter();
		if (emitter == NULL || !emitter->getClass()->derivesFrom(MTS_CLASS(MapEmitter))) {
			Log(EError, "The environment emitter must be of the MapEmitter class.");
			return false;
		}

		size_t numResults = targets.size() * numHarmonics;
		size_t numCores = Scheduler::getInstance()->getCoreCount();

		MapEmitter* map = static_cast<MapEmitter*>(const_cast<Emitter*>(emitter));
		ref<TransferWorker>* workers = new ref<TransferWorker>[numCores];
		results = new Spectrum[numResults];

		for (size_t i = 0; i < numResults; i++)
			results[i] = Spectrum(0.0f);

		for (harmonic = 0; harmonic < numHarmonics; harmonic++) {
			for (direction = harmonic > 0 ? -1 : 1; direction < 2; direction+=2) {
				ostringstream name;
				name << mapsRoot << "/" << harmonic << " " << direction << ".exr";
				map->setFilename(fs::path(name.str()));

				for (size_t i = 0; i < numCores; i++) {
					workers[i] = new TransferWorker(this, i);
					workers[i]->start();
				}

				for (size_t i = 0; i < numCores; i++)
					workers[i]->join();
			}
		}

		// Check results
		int count = 0;
		for (size_t i = 0; i < targets.size(); ++i)
			if (results[i * numHarmonics].isZero())
				count++;

		if (count > 0)
			Log(EInfo, "%d points returned black", count);

		// Write file
		fs::path path = fs::path(scene->getDestinationFile().string());
		path.replace_extension(".transfer");
		ref<FileStream> file = new FileStream(path, FileStream::ETruncWrite);
		file->write(&numHarmonics, sizeof(int));

		for (size_t i = 0; i < numResults; ++i)
			for (int k = 0; k < Spectrum::dim; k++)
				file->write(&results[i][k], sizeof(float));

		Log(EInfo, "Written transfer to \"%s\"", path.string().c_str());
		return true;
	}

	std::string mapsRoot;

	int numHarmonics;
	int harmonic, direction;
	vector<Target> targets;
	ref<Spectrum> results;
	ref<Medium> medium;
	ref<Scene> scene;

MTS_DECLARE_CLASS()
};

TransferWorker::TransferWorker(VertexTransfer* p, int i) : Thread("TransferWorker"), parent(p), index(i) {}

void TransferWorker::run() {
	ref<Sampler> sampler = parent->scene->getSampler()->clone();
	RadianceQueryRecord rRec(parent->scene, sampler);
	Intersection& its = rRec.its;

	int numSamples = sampler->getSampleCount();
	int numWorkers = Scheduler::getInstance()->getCoreCount();
	int numTargets = parent->targets.size();
	int step = numTargets / numWorkers;
	
	int first = step * index++;
	int last = (index == numWorkers) ? numTargets : step * index;

	for (int i = first; i < last; i++) {
		Target& target = parent->targets[i];
		RayDifferential ray(target.point + target.normal, -target.normal, 0);
		Frame frame(target.normal);
		Spectrum result(0.0f);
		float hits = 0.0f;

		sampler->generate(Point2i(-1));

		for (int k = 0; k < numSamples; ++k) {
			rRec.newQuery(RadianceQueryRecord::ERadiance ^ RadianceQueryRecord::EIntersection, parent->medium);
			rRec.dist = its.t = 0;

			its.shape = target.shape;
			its.shFrame = its.geoFrame = frame;
			its.uv = target.uv;
			its.p = target.point;
			its.wi = its.toLocal(-ray.d);
			its.hasUVPartials = 0;
			its.time = 0;

			Spectrum sample = parent->Li(ray, rRec);
			if (result.isValid()) {
				result += sample;
				hits++;
			}

			sampler->advance();
		}

  		parent->results[i * parent->numHarmonics + parent->harmonic] += result / hits * parent->direction;
	}
}

MTS_IMPLEMENT_CLASS(VertexTransfer, false, LayeredIntegrator)
MTS_EXPORT_PLUGIN(VertexTransfer, "Outputs transfer per vertex to a binary file.");
MTS_NAMESPACE_END