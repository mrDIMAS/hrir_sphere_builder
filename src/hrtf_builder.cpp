/// Small command line tool to create HRIR Sphere from IRCAM HRIR database.
///
/// For debug you can use:
/// 
/// #define VISUAL_DEBUG 
/// 
/// to force tool to emit *.obj file with sphere geometry to ensure its correctness
/// 
/// MIT License. Dmitry Stepanov, 2019.

#include <iostream>
#include <fstream>
#include <experimental/filesystem>

#define CONVHULL_3D_ENABLE
#include "convexhull_3d/convhull_3d.h"

#include "AudioFile/AudioFile.h"

using namespace std;
namespace fs = std::experimental::filesystem::v1;

template<typename T>
void WriteExact(ofstream& stream, const T& v)
{
	stream.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template<typename T>
void WriteExact(ofstream& stream, const vector<T>& v)
{
	stream.write(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(T));
}

struct Vec3 {
	float x;
	float y;
	float z;

	Vec3() :x(0), y(0), z(0)
	{
	}

	Vec3(float x, float y, float z) : x(x), y(y), z(z)
	{
	}
};

struct HrtfVertex {
	HrtfVertex(uint32_t sampleRate, const Vec3& position, vector<float>&& leftHRIR, vector<float>&& rightHRIR)
		: m_Position(position),
		m_SampleRate(sampleRate),
		m_LeftHRIR(std::move(leftHRIR)),
		m_RightHRIR(std::move(rightHRIR))
	{
	}

	uint32_t m_SampleRate;
	Vec3 m_Position;
	vector<float> m_LeftHRIR;
	vector<float> m_RightHRIR;
};

static char FileMagic[4] = { 'H', 'R', 'I', 'R' };

class HrtfSphere {
public:
	HrtfSphere() = default;
	~HrtfSphere() = default;

	void AddVertex(HrtfVertex&& v)
	{
		m_Vertices.push_back(std::move(v));
	}

	void Triangulate()
	{
		vector<ch_vertex> vertices;
		for (const auto& v : m_Vertices) {
			ch_vertex ch_v;
			ch_v.x = v.m_Position.x;
			ch_v.y = v.m_Position.y;
			ch_v.z = v.m_Position.z;
			vertices.push_back(ch_v);
		}

		int* outIndices = NULL;
		int faceCount = 0;
		convhull_3d_build(vertices.data(), static_cast<int>(vertices.size()), &outIndices, &faceCount);

		m_Indices.clear();
		for (int i = 0; i < faceCount * 3; ++i) {
			m_Indices.push_back(outIndices[i]);
		}

	#ifdef VISUAL_DEBUG
		convhull_3d_export_obj(vertices.data(), static_cast<int>(vertices.size()), outIndices, faceCount, false, "test");
	#endif
	}

	void Validate()
	{
		if (m_Vertices.empty()) {
			throw runtime_error("sphere is empty!");
		}

		const auto expectedHrirLen = m_Vertices[0].m_LeftHRIR.size();
		const auto expectedSampleRate = m_Vertices[0].m_SampleRate;
		for (const auto& v : m_Vertices) {
			if (v.m_LeftHRIR.size() != expectedHrirLen || v.m_RightHRIR.size() != expectedHrirLen) {
				throw runtime_error("HRIR length must be same across all files!");
			}
			if (v.m_SampleRate != expectedSampleRate) {
				throw runtime_error("HRIR must have same sample rate across all files!");
			}
		}
	}

	void Save(ofstream& file)
	{
		const uint32_t sampleRate = m_Vertices[0].m_SampleRate;
		const auto hrirLen = static_cast<uint32_t>(m_Vertices[0].m_LeftHRIR.size());
		const auto vertexCount = static_cast<uint32_t>(m_Vertices.size());
		const auto indexCount = static_cast<uint32_t>(m_Indices.size());

		// Header
		WriteExact(file, FileMagic);
		WriteExact(file, sampleRate);
		WriteExact(file, hrirLen);
		WriteExact(file, vertexCount);
		WriteExact(file, indexCount);

		// Index buffer
		WriteExact(file, m_Indices);

		// Vertices
		for (const auto& v : m_Vertices) {
			WriteExact(file, v.m_Position.x);
			WriteExact(file, v.m_Position.y);
			WriteExact(file, v.m_Position.z);

			WriteExact(file, v.m_LeftHRIR);
			WriteExact(file, v.m_RightHRIR);
		}

		file.flush();
	}

private:
	vector<HrtfVertex> m_Vertices;
	vector<uint32_t> m_Indices;
};

Vec3 SphericalToCartesian(float azimuth, float elevation, float radius)
{
	// Translates spherical to cartesian, where Y - up, Z - forward, X - right
	const float x = radius * sin(elevation) * sin(azimuth);
	const float y = radius * cos(elevation);
	const float z = -radius * sin(elevation) * cos(azimuth);
	return Vec3(x, y, z);
}

constexpr float Pi = 3.1415926535f;

float ToRadians(float degrees)
{
	return degrees / 180.0f * Pi;
}

Vec3 ParseFileName(const string& fileName)
{
	const auto azimuth_location = fileName.find("_T");
	if (azimuth_location == string::npos) {
		throw runtime_error("invalid file name");
	}
	const auto azimuth = static_cast<float>(atof(fileName.substr(azimuth_location + 2, 3).c_str()));

	const auto elevation_location = fileName.find("_P");
	if (elevation_location == string::npos) {
		throw runtime_error("invalid file name");
	}
	const auto elevation = 90.0f - static_cast<float>(atof(fileName.substr(elevation_location + 2, 3).c_str()));

	return SphericalToCartesian(ToRadians(azimuth), ToRadians(elevation), 1.0);
}

int main(int argc, char** arcv) try {
	if (argc != 2) {
		throw runtime_error("no path specified");
	}

	const char* folder = arcv[1];

	if (!fs::is_directory(folder)) {
		throw runtime_error("path must be a folder!");
	}

	HrtfSphere sphere;

	for (const auto& entry : fs::directory_iterator(folder)) {
		const auto path = entry.path().u8string();

		cout << "\rworking on " << path << flush;

		const auto position = ParseFileName(path);

		AudioFile<float> buffer; 

		if (!buffer.load(path)) {
			throw runtime_error("invalid wav");
		}

		if (buffer.getNumChannels() != 2) {
			throw runtime_error("hrir must be two channel wav file");
		}
		
		sphere.AddVertex(HrtfVertex(buffer.getSampleRate(), position, std::move(buffer.samples[0]), std::move(buffer.samples[1])));
	}

	sphere.Validate();
	sphere.Triangulate();

	const auto folderPath = fs::path(folder);
	const auto outputPath = folderPath.parent_path().append(folderPath.filename().replace_extension(".bin"));

	ofstream output(outputPath, ios::binary);
	sphere.Save(output);

	cout << "\ndone. saved into " << outputPath << endl;

} catch (exception& e) {
	cerr << e.what();
}