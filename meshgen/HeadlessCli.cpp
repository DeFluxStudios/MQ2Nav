//
// HeadlessCli.cpp
//
// Headless MeshGenerator commands. Every command prints one JSON object on stdout; progress and
// diagnostics go to stderr. Nothing here writes to an existing file.
//

#include "meshgen/HeadlessCli.h"

#include "common/NavMesh.h"
#include "common/NavMeshData.h"
#include "meshgen/InputGeom.h"
#include "meshgen/MapGeometryLoader.h"
#include "meshgen/NavMeshTool.h"

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourNode.h>
#include <Recast.h>
#include <glm/gtc/type_ptr.hpp>
#include <fmt/format.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <concrt.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <tuple>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <Windows.h>
#include <bcrypt.h>

#pragma comment(lib, "bcrypt.lib")

namespace fs = std::filesystem;

namespace
{
	// Exit codes: 0 ok, 2 usage, 3 missing input, 4 build or write failure, 5 bad mesh,
	// 6 a check failed (diff found lost fixes, or a route regressed), 70 internal error.
	class CliError : public std::runtime_error
	{
	public:
		CliError(int exitCode, const std::string& code, const std::string& message)
			: std::runtime_error(message)
			, exitCode(exitCode)
			, code(code)
		{
		}

		int exitCode;
		std::string code;
	};

	class ConsoleRecastContext : public rcContext
	{
	public:
		ConsoleRecastContext()
			: rcContext(false)
		{
			enableLog(true);
		}

	protected:
		void doLog(const rcLogCategory category, const char* message, const int length) override
		{
			auto logger = spdlog::default_logger();
			if (!logger)
				return;

			spdlog::level::level_enum level = spdlog::level::info;
			if (category == RC_LOG_WARNING)
				level = spdlog::level::warn;
			else if (category == RC_LOG_ERROR)
				level = spdlog::level::err;

			logger->log(level, spdlog::string_view_t(message, static_cast<size_t>(length)));
		}
	};

	using Options = std::unordered_map<std::string, std::string>;
	using Json = rapidjson::Writer<rapidjson::StringBuffer>;

	// The plugin's own search box and limits (plugin/NavigationPath.h and .cpp).
	const float kPluginExtents[3] = { 5.f, 10.f, 5.f };
	const int kMaxStraightPath = 16384;
	const float kNodePoolGrowth = 1.5f;
	const int kNodePoolMaxSize = 1024 * 1024;

	//------------------------------------------------------------------------
	// Output helpers

	void Emit(const rapidjson::StringBuffer& buffer)
	{
		fmt::print("{}\n", buffer.GetString());
	}

	void PrintError(const std::string& code, const std::string& message)
	{
		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(false);
		w.Key("error"); w.String(code.c_str());
		w.Key("message"); w.String(message.c_str());
		w.EndObject();
		Emit(buffer);
	}

	double Round1(float value)
	{
		return std::round(static_cast<double>(value) * 10.0) / 10.0;
	}

	// Mesh axes are x, y (up), z. /loc order is y, x, z with z up, so a mesh point prints as [z, x, y].
	template <typename Writer>
	void WriteLoc(Writer& w, const float* p)
	{
		w.StartArray();
		w.Double(Round1(p[2]));
		w.Double(Round1(p[0]));
		w.Double(Round1(p[1]));
		w.EndArray();
	}

	template <typename Writer>
	void WriteVec(Writer& w, const glm::vec3& p)
	{
		w.StartArray();
		w.Double(p.x);
		w.Double(p.y);
		w.Double(p.z);
		w.EndArray();
	}

	void PrintUsage()
	{
		fmt::print("{}",
			"MeshGenerator headless commands (every command prints one JSON object on stdout):\n"
			"\n"
			"  build --zone Z --eq-path PATH --stage-root PATH [--mesh-root PATH] [--no-max-extents]\n"
			"      Settings: [--preset mq2nav|triune] [--settings-from MESH] then any of\n"
			"        --tile-size --cell-size --cell-height --agent-height --agent-radius --max-climb\n"
			"        --max-slope --region-min-size --region-merge-size --edge-max-len --edge-max-error\n"
			"        --verts-per-poly --detail-sample-dist --detail-sample-max-error\n"
			"      Fixes:    [--carry-from MESH] [--fixes FILE] [--connections FILE] [--volumes FILE]\n"
			"      Region:   [--base MESH [--box \"Y1,X1:Y2,X2\"]]  rebuild only those tiles and the tiles\n"
			"                new links or volumes touch; settings come from the base mesh\n"
			"      Other:    [--walkways] [--auto-drops FILE [--max-drop N] [--drop-spacing N]]\n"
			"                [--threads N] [--mesh-version 4|5] [--allow-live]\n"
			"      A stage root inside a MacroQuest install is refused unless --allow-live is given.\n"
			"  validate --zone Z --mesh PATH\n"
			"  export   --zone Z --mesh PATH --out FILE        links, volumes, areas and settings as JSON\n"
			"  diff     --zone Z --old MESH --new MESH [--tolerance N] [--strict]   exit 6 when fixes were lost;\n"
			"           --strict also fails on changed settings or tile geometry\n"
			"  path     --zone Z --mesh PATH --from \"Y,X,Z\" --to \"Y,X,Z\" [--extents X,Y,Z]\n"
			"  reach    --zone Z --mesh PATH --from \"Y,X,Z\" [--min-polys N] [--limit N] [--extents X,Y,Z]\n"
			"  inspect  --zone Z --mesh PATH --at \"Y,X,Z\" [--radius N] [--extents X,Y,Z]\n"
			"  probe    --zone Z --mesh PATH (--points \"Y,X,Z;Y,X,Z\" | --routes FILE) [--extents X,Y,Z]\n"
			"           routes: name, points, optional expect, maxLength, endZ and endTolerance (default 5)\n"
			"  config   [--ini PATH]                            what the GUI's MeshGenerator.ini resolves to\n"
			"\n"
			"Points are /loc order (Y,X,Z). JSON files use mesh axes [x,y,z] with y up, as export writes them.\n"
			"Build writes <stage-root>\\MQ2Nav\\<zone>.navmesh and never overwrites an existing file.\n"
			"Exit codes: 0 ok, 2 usage, 3 missing input, 4 build or write failure, 5 bad mesh,\n"
			"6 check failed (diff lost fixes, a probe route failed, path found no full route,\n"
			"reach listed unreachable islands), 70 internal error.\n");
	}

	//------------------------------------------------------------------------
	// Argument helpers

	Options ParseOptions(int argc, char* argv[], const std::set<std::string>& valueOptions,
		const std::set<std::string>& flagOptions)
	{
		Options options;
		for (int index = 2; index < argc; ++index)
		{
			const std::string key = argv[index];
			if (key == "--help" || key == "-h")
			{
				PrintUsage();
				throw CliError(0, "help", "");
			}

			if (key.size() < 3 || key.substr(0, 2) != "--")
				throw CliError(2, "invalid_argument", "Expected a named option, got: " + key);

			if (flagOptions.count(key))
			{
				if (!options.emplace(key, "").second)
					throw CliError(2, "invalid_argument", "Option provided more than once: " + key);
				continue;
			}

			if (!valueOptions.count(key))
				throw CliError(2, "invalid_argument", "Unknown option: " + key);
			if (index + 1 >= argc || std::string(argv[index + 1]).rfind("--", 0) == 0)
				throw CliError(2, "invalid_argument", "Missing value for option: " + key);
			if (!options.emplace(key, argv[++index]).second)
				throw CliError(2, "invalid_argument", "Option provided more than once: " + key);
		}
		return options;
	}

	std::string RequiredOption(const Options& options, const std::string& name)
	{
		auto iter = options.find(name);
		if (iter == options.end() || iter->second.empty())
			throw CliError(2, "invalid_argument", "Required option missing: " + name);
		return iter->second;
	}

	fs::path PathOption(const Options& options, const std::string& name)
	{
		return fs::absolute(RequiredOption(options, name)).lexically_normal();
	}

	std::string GetZone(const Options& options)
	{
		std::string zone = RequiredOption(options, "--zone");
		for (char& character : zone)
		{
			if (!((character >= 'a' && character <= 'z')
				|| (character >= 'A' && character <= 'Z')
				|| (character >= '0' && character <= '9')
				|| character == '_'))
			{
				throw CliError(2, "invalid_zone", "Zone short name may contain only letters, digits, and underscores.");
			}
			if (character >= 'A' && character <= 'Z')
				character = static_cast<char>(character - 'A' + 'a');
		}
		return zone;
	}

	float NumberOption(const Options& options, const std::string& name, float fallback, float low, float high)
	{
		const auto found = options.find(name);
		if (found == options.end())
			return fallback;
		float value = 0.0f;
		try
		{
			size_t used = 0;
			value = std::stof(found->second, &used);
			if (used != found->second.size())
				throw std::invalid_argument(name);
		}
		catch (const std::exception&)
		{
			throw CliError(2, "bad_option", name + " must be a number.");
		}
		if (!std::isfinite(value) || value < low || value > high)
			throw CliError(2, "bad_option", fmt::format("{} must be from {} to {}.", name, low, high));
		return value;
	}

	// Three numbers separated by commas, all finite.
	bool ParseTriple(const std::string& text, float out[3])
	{
		if (sscanf_s(text.c_str(), "%f,%f,%f", &out[0], &out[1], &out[2]) != 3)
			return false;
		return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
	}

	// /loc "Y,X,Z" to mesh axes.
	glm::vec3 ParseLoc(const std::string& text, const std::string& name)
	{
		float v[3];
		if (!ParseTriple(text, v))
			throw CliError(2, "bad_points", name + " must be Y,X,Z in /loc order.");
		return glm::vec3(v[1], v[2], v[0]);
	}

	std::vector<glm::vec3> ParseLocList(const std::string& text, const std::string& name)
	{
		std::vector<glm::vec3> result;
		std::stringstream stream(text);
		std::string item;
		while (std::getline(stream, item, ';'))
			result.push_back(ParseLoc(item, name));
		if (result.empty())
			throw CliError(2, "bad_points", name + " must be Y,X,Z;Y,X,Z in /loc order.");
		return result;
	}

	void GetExtents(const Options& options, float out[3])
	{
		dtVcopy(out, kPluginExtents);
		const auto found = options.find("--extents");
		if (found == options.end())
			return;
		if (!ParseTriple(found->second, out) || out[0] <= 0 || out[1] <= 0 || out[2] <= 0)
			throw CliError(2, "bad_option", "--extents must be three positive numbers X,Y,Z in mesh axes (Y is up).");
	}

	rapidjson::Document ReadJsonFile(const std::string& path, const std::string& code)
	{
		std::ifstream file(path, std::ios::binary);
		if (!file)
			throw CliError(3, "missing_" + code, "File not found: " + path);
		const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		rapidjson::Document document;
		document.Parse(text.c_str());
		if (document.HasParseError())
			throw CliError(2, "bad_" + code, "Not valid JSON: " + path);
		return document;
	}

	// SHA-256 of a file as upper-case hex, or "" when it cannot be read.
	std::string FileSha256(const fs::path& path)
	{
		std::ifstream file(path, std::ios::binary);
		if (!file)
			return "";
		BCRYPT_ALG_HANDLE alg = nullptr;
		BCRYPT_HASH_HANDLE hash = nullptr;
		if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0) != 0)
			return "";
		std::string result;
		if (BCryptCreateHash(alg, &hash, nullptr, 0, nullptr, 0, 0) == 0)
		{
			std::vector<char> chunk(1 << 16);
			bool ok = true;
			while (ok && file)
			{
				file.read(chunk.data(), chunk.size());
				const auto got = file.gcount();
				if (got > 0)
					ok = BCryptHashData(hash, reinterpret_cast<PUCHAR>(chunk.data()), static_cast<ULONG>(got), 0) == 0;
			}
			unsigned char digest[32];
			if (ok && BCryptFinishHash(hash, digest, sizeof(digest), 0) == 0)
				for (unsigned char b : digest)
					result += fmt::format("{:02X}", b);
			BCryptDestroyHash(hash);
		}
		BCryptCloseAlgorithmProvider(alg, 0);
		return result;
	}

	void WriteNewFile(const fs::path& path, const std::string& text)
	{
		std::error_code error;
		if (fs::exists(path, error))
			throw CliError(4, "output_exists", "Refusing to overwrite: " + path.string());
		if (path.has_parent_path())
			fs::create_directories(path.parent_path(), error);
		std::ofstream file(path, std::ios::binary);
		if (!file || !(file << text))
			throw CliError(4, "write_failed", "Could not write: " + path.string());
	}

	//------------------------------------------------------------------------
	// Settings

	struct Setting
	{
		const char* name;
		const char* key;
		float NavMeshConfig::* field;
		float low;
		float high;
		bool whole;
	};

	// Limits follow the MeshGenerator sliders. Detail sample distance is the exception: the slider
	// stops at 0.9 but MQ2Nav's default is 6, and Recast treats anything under 0.9 as off.
	const Setting kSettings[] = {
		{ "--tile-size", "tileSize", &NavMeshConfig::tileSize, 16.0f, 1024.0f, true },
		{ "--cell-size", "cellSize", &NavMeshConfig::cellSize, 0.1f, 1.0f, false },
		{ "--cell-height", "cellHeight", &NavMeshConfig::cellHeight, 0.1f, 1.0f, false },
		{ "--agent-height", "agentHeight", &NavMeshConfig::agentHeight, 0.1f, 15.0f, false },
		{ "--agent-radius", "agentRadius", &NavMeshConfig::agentRadius, 0.1f, 15.0f, false },
		{ "--max-climb", "agentMaxClimb", &NavMeshConfig::agentMaxClimb, 0.1f, 15.0f, false },
		{ "--max-slope", "agentMaxSlope", &NavMeshConfig::agentMaxSlope, 0.0f, 90.0f, false },
		{ "--region-min-size", "regionMinSize", &NavMeshConfig::regionMinSize, 0.0f, 150.0f, false },
		{ "--region-merge-size", "regionMergeSize", &NavMeshConfig::regionMergeSize, 0.0f, 150.0f, false },
		{ "--edge-max-len", "edgeMaxLen", &NavMeshConfig::edgeMaxLen, 0.0f, 50.0f, false },
		{ "--edge-max-error", "edgeMaxError", &NavMeshConfig::edgeMaxError, 0.1f, 3.0f, false },
		{ "--verts-per-poly", "vertsPerPoly", &NavMeshConfig::vertsPerPoly, 3.0f, 12.0f, true },
		{ "--detail-sample-dist", "detailSampleDist", &NavMeshConfig::detailSampleDist, 0.0f, 32.0f, false },
		{ "--detail-sample-max-error", "detailSampleMaxError", &NavMeshConfig::detailSampleMaxError, 0.0f, 100.0f, false },
	};

	bool HasSettingFlag(const Options& options)
	{
		for (const Setting& setting : kSettings)
			if (options.count(setting.name))
				return true;
		return false;
	}

	void ApplySettingFlags(NavMeshConfig& config, const Options& options)
	{
		for (const Setting& setting : kSettings)
		{
			const auto found = options.find(setting.name);
			if (found == options.end())
				continue;
			float value = 0.0f;
			try
			{
				size_t used = 0;
				value = std::stof(found->second, &used);
				if (used != found->second.size())
					throw std::invalid_argument(setting.name);
			}
			catch (const std::exception&)
			{
				throw CliError(2, "bad_setting", std::string("Setting must be a number: ") + setting.name);
			}
			if (!std::isfinite(value) || value < setting.low || value > setting.high
				|| (setting.whole && value != std::floor(value)))
			{
				throw CliError(2, "bad_setting", fmt::format("{} must be {}from {} to {}.", setting.name,
					setting.whole ? "a whole number " : "", setting.low, setting.high));
			}
			config.*setting.field = value;
		}
	}

	NavMeshConfig PresetConfig(const std::string& name)
	{
		NavMeshConfig config;
		if (name == "mq2nav")
			return config;
		if (name == "triune")
		{
			// The settings of the live Everfrost mesh on 2026-11-25.
			config.tileSize = 159;
			config.cellSize = 0.6f;
			config.cellHeight = 0.3f;
			config.agentHeight = 8.0f;
			config.agentRadius = 3.0f;
			config.agentMaxClimb = 3.0f;
			config.agentMaxSlope = 75.0f;
			return config;
		}
		throw CliError(2, "bad_preset", "--preset must be mq2nav or triune.");
	}

	template <typename Writer>
	void WriteSettings(Writer& w, const NavMeshConfig& config)
	{
		w.StartObject();
		for (const Setting& setting : kSettings)
		{
			w.Key(setting.key);
			w.Double(std::round(static_cast<double>(config.*setting.field) * 10000.0) / 10000.0);
		}
		w.Key("partitionType"); w.Int(static_cast<int>(config.partitionType));
		w.EndObject();
	}

	//------------------------------------------------------------------------
	// Mesh helpers

	std::unique_ptr<NavMesh> LoadMesh(const std::string& zone, const fs::path& path)
	{
		std::error_code error;
		if (!fs::is_regular_file(path, error))
			throw CliError(3, "missing_mesh", "Mesh file not found: " + path.string());
		auto mesh = std::make_unique<NavMesh>("", zone);
		const NavMesh::LoadResult result = mesh->LoadNavMeshFile(path.string());
		if (result != NavMesh::LoadResult::Success || !mesh->GetNavMesh())
			throw CliError(5, "invalid_mesh", fmt::format("Mesh could not be loaded (load result {}): {}",
				static_cast<int>(result), path.string()));
		return mesh;
	}

	dtQueryFilter MakeFilter(NavMesh& navMesh)
	{
		// The plugin's filter (plugin/NavigationPath.cpp): every flag but Disabled, with the
		// mesh's own area costs.
		dtQueryFilter filter;
		filter.setIncludeFlags(+PolyFlags::All);
		filter.setExcludeFlags(+PolyFlags::Disabled);
		navMesh.FillFilterAreaCosts(filter);
		return filter;
	}

	template <typename Fn>
	void ForEachPoly(const dtNavMesh* mesh, Fn&& fn)
	{
		for (int i = 0; i < mesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = mesh->getTile(i);
			if (!tile || !tile->header)
				continue;
			const dtPolyRef base = mesh->getPolyRefBase(tile);
			for (int p = 0; p < tile->header->polyCount; ++p)
				fn(base | static_cast<dtPolyRef>(p), tile, &tile->polys[p]);
		}
	}

	int CountTiles(const dtNavMesh* mesh)
	{
		if (!mesh)
			return 0;
		int count = 0;
		for (int i = 0; i < mesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = mesh->getTile(i);
			if (tile && tile->header)
				++count;
		}
		return count;
	}

	int CountPolys(const dtNavMesh* mesh)
	{
		int count = 0;
		if (mesh)
			ForEachPoly(mesh, [&](dtPolyRef, const dtMeshTile*, const dtPoly*) { ++count; });
		return count;
	}

	std::array<float, 3> PolyCentre(const dtMeshTile* tile, const dtPoly* poly)
	{
		std::array<float, 3> c{ 0, 0, 0 };
		for (int v = 0; v < poly->vertCount; ++v)
			for (int a = 0; a < 3; ++a)
				c[a] += tile->verts[poly->verts[v] * 3 + a] / poly->vertCount;
		return c;
	}

	float PolyAreaXZ(const dtMeshTile* tile, const dtPoly* poly)
	{
		float sum = 0;
		for (int v = 0; v < poly->vertCount; ++v)
		{
			const float* a = &tile->verts[poly->verts[v] * 3];
			const float* b = &tile->verts[poly->verts[(v + 1) % poly->vertCount] * 3];
			sum += a[0] * b[2] - b[0] * a[2];
		}
		return std::fabs(sum) * 0.5f;
	}

	// Every poly reachable from `from` along links that pass the filter.
	std::unordered_set<dtPolyRef> Flood(const dtNavMesh* mesh, const dtQueryFilter& filter, dtPolyRef from,
		const std::unordered_set<dtPolyRef>* skip = nullptr)
	{
		std::unordered_set<dtPolyRef> seen{ from };
		std::vector<dtPolyRef> open{ from };
		while (!open.empty())
		{
			const dtPolyRef cur = open.back();
			open.pop_back();
			const dtMeshTile* tile = nullptr;
			const dtPoly* poly = nullptr;
			mesh->getTileAndPolyByRefUnsafe(cur, &tile, &poly);
			for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = tile->links[l].next)
			{
				const dtPolyRef next = tile->links[l].ref;
				if (!next || seen.count(next) || (skip && skip->count(next)))
					continue;
				const dtMeshTile* nextTile = nullptr;
				const dtPoly* nextPoly = nullptr;
				mesh->getTileAndPolyByRefUnsafe(next, &nextTile, &nextPoly);
				if (!filter.passFilter(next, nextTile, nextPoly))
					continue;
				seen.insert(next);
				open.push_back(next);
			}
		}
		return seen;
	}

	//------------------------------------------------------------------------
	// Path planning, the same way the plugin does it (plugin/NavigationPath.cpp RecomputePath).

	struct PathResult
	{
		std::string status;
		std::vector<float> verts;
		std::vector<unsigned char> flags;
		std::vector<dtPolyRef> polys;
		float length = 0;
		int links = 0;
	};

	PathResult PlanPath(dtNavMeshQuery& query, const dtQueryFilter& filter, const float extents[3],
		const glm::vec3& from, const glm::vec3& to)
	{
		PathResult result;
		dtPolyRef startRef = 0, endRef = 0;
		float spos[3], epos[3];
		query.findNearestPoly(glm::value_ptr(from), extents, &filter, &startRef, spos);
		if (!startRef)
		{
			result.status = "start_not_on_mesh";
			return result;
		}
		query.findNearestPoly(glm::value_ptr(to), extents, &filter, &endRef, epos);
		if (!endRef)
		{
			result.status = "end_not_on_mesh";
			return result;
		}

		int polysSize = kMaxStraightPath;
		std::vector<dtPolyRef> polys(polysSize);
		int numPolys = 0;
		dtStatus status = 0;
		for (int iters = 0; iters < 100; ++iters)
		{
			status = query.findPath(startRef, endRef, spos, epos, &filter, polys.data(), &numPolys, polysSize);
			bool retry = false;
			if (dtStatusDetail(status, DT_OUT_OF_NODES))
			{
				const uint32_t maxNodes = static_cast<uint32_t>(query.getNodePool()->getMaxNodes());
				const uint32_t newMaxNodes = (std::min<uint32_t>)(static_cast<uint32_t>(maxNodes * kNodePoolGrowth),
					(std::min<uint32_t>)(DT_NULL_IDX, 1 << DT_NODE_PARENT_BITS) - 1);
				if (maxNodes != newMaxNodes && newMaxNodes < static_cast<uint32_t>(kNodePoolMaxSize))
				{
					query.init(query.getAttachedNavMesh(), newMaxNodes);
					retry = true;
				}
			}
			if (dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
			{
				const int newSize = static_cast<int>(polysSize * kNodePoolGrowth);
				if (newSize < kNodePoolMaxSize)
				{
					polysSize = newSize;
					polys.assign(polysSize, 0);
					retry = true;
				}
			}
			if (!retry)
				break;
		}

		if (dtStatusFailed(status))
			result.status = "failed";
		else if (dtStatusDetail(status, DT_OUT_OF_NODES) || dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
			result.status = "too_far";
		else if ((numPolys > 0 && polys[numPolys - 1] != endRef) || dtStatusDetail(status, DT_PARTIAL_RESULT))
			result.status = "partial";
		if (!result.status.empty() || numPolys == 0)
		{
			if (result.status.empty())
				result.status = "failed";
			return result;
		}

		result.verts.resize(kMaxStraightPath * 3);
		result.flags.resize(kMaxStraightPath);
		result.polys.resize(kMaxStraightPath);
		int count = 0;
		query.findStraightPath(spos, epos, polys.data(), numPolys, result.verts.data(), result.flags.data(),
			result.polys.data(), &count, kMaxStraightPath, DT_STRAIGHTPATH_AREA_CROSSINGS);
		result.verts.resize(count * 3);
		result.flags.resize(count);
		result.polys.resize(count);
		for (int i = 1; i < count; ++i)
			result.length += dtVdist(&result.verts[(i - 1) * 3], &result.verts[i * 3]);
		for (int i = 0; i < count; ++i)
			if (result.flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				++result.links;
		result.status = "found";
		return result;
	}

	//------------------------------------------------------------------------
	// Fixes: off-mesh links, area volumes and area types, read and written as JSON.

	bool ReadVec(const rapidjson::Value& item, const char* key, glm::vec3& out)
	{
		if (!item.HasMember(key) || !item[key].IsArray() || item[key].Size() != 3)
			return false;
		for (rapidjson::SizeType i = 0; i < 3; ++i)
		{
			if (!item[key][i].IsNumber())
				return false;
			out[i] = item[key][i].GetFloat();
			if (!std::isfinite(out[i]))
				return false;
		}
		return true;
	}

	// Area ids 62 and 63 are reserved: 62 is the walkway marker during a build and 63 is Recast's
	// walkable area. Volumes may use 0 (not walkable); links and area types may not.
	uint8_t ReadAreaId(const rapidjson::Value& item, const std::string& code, uint8_t fallback, unsigned int lowest)
	{
		if (!item.HasMember("area"))
			return fallback;
		if (!item["area"].IsUint() || item["area"].GetUint() < lowest || item["area"].GetUint() > 61)
			throw CliError(2, code, fmt::format("area must be a whole number from {} to 61.", lowest));
		return static_cast<uint8_t>(item["area"].GetUint());
	}

	int AddConnections(const rapidjson::Value& array, NavMesh& navMesh, std::vector<uint32_t>& added)
	{
		if (!array.IsArray())
			throw CliError(2, "bad_connections", "Connections must be a JSON array.");
		int count = 0;
		for (const auto& item : array.GetArray())
		{
			auto connection = std::make_unique<OffMeshConnection>();
			if (!item.IsObject() || !ReadVec(item, "start", connection->start) || !ReadVec(item, "end", connection->end))
				throw CliError(2, "bad_connections", "Each connection needs start and end as [x,y,z].");
			if (item.HasMember("bidirectional") && item["bidirectional"].IsBool())
				connection->bidirectional = item["bidirectional"].GetBool();
			if (item.HasMember("name") && item["name"].IsString())
				connection->name = item["name"].GetString();
			connection->areaType = ReadAreaId(item, "bad_connections", connection->areaType, 1);
			added.push_back(navMesh.AddConnection(std::move(connection))->id);
			++count;
		}
		return count;
	}

	int AddVolumes(const rapidjson::Value& array, NavMesh& navMesh, std::vector<uint32_t>& added)
	{
		if (!array.IsArray())
			throw CliError(2, "bad_volumes", "Volumes must be a JSON array.");
		int count = 0;
		for (const auto& item : array.GetArray())
		{
			if (!item.IsObject() || !item.HasMember("verts") || !item["verts"].IsArray() || item["verts"].Size() < 3
				|| !item.HasMember("hmin") || !item["hmin"].IsNumber() || !item.HasMember("hmax") || !item["hmax"].IsNumber()
				|| !item.HasMember("area"))
				throw CliError(2, "bad_volumes", "Each volume needs verts (3 or more), hmin, hmax and area.");
			std::vector<glm::vec3> verts;
			for (const auto& v : item["verts"].GetArray())
			{
				if (!v.IsArray() || v.Size() != 3 || !v[0].IsNumber() || !v[1].IsNumber() || !v[2].IsNumber())
					throw CliError(2, "bad_volumes", "Volume vertices must be [x,y,z].");
				verts.emplace_back(v[0].GetFloat(), v[1].GetFloat(), v[2].GetFloat());
			}
			const uint8_t area = ReadAreaId(item, "bad_volumes", 0, 0);
			const float hmin = item["hmin"].GetFloat(), hmax = item["hmax"].GetFloat();
			if (!(hmin < hmax))
				throw CliError(2, "bad_volumes", "Volume hmin must be below hmax.");
			const std::string name = item.HasMember("name") && item["name"].IsString() ? item["name"].GetString() : "";
			added.push_back(navMesh.AddConvexVolume(verts, name, hmin, hmax, area)->id);
			++count;
		}
		return count;
	}

	int AddAreas(const rapidjson::Value& array, NavMesh& navMesh)
	{
		if (!array.IsArray())
			throw CliError(2, "bad_areas", "Areas must be a JSON array.");
		int count = 0;
		for (const auto& item : array.GetArray())
		{
			if (!item.IsObject() || !item.HasMember("id") || !item["id"].IsUint() || item["id"].GetUint() < 1 || item["id"].GetUint() > 61
				|| !item.HasMember("name") || !item["name"].IsString() || !item.HasMember("cost") || !item["cost"].IsNumber())
				throw CliError(2, "bad_areas", "Each area needs id (1 to 61), name and cost.");
			PolyAreaType area{};
			area.id = static_cast<uint8_t>(item["id"].GetUint());
			area.name = item["name"].GetString();
			area.cost = item["cost"].GetFloat();
			area.color = item.HasMember("color") && item["color"].IsUint() ? item["color"].GetUint() : 0xffffffff;
			area.flags = item.HasMember("flags") && item["flags"].IsUint() ? static_cast<uint16_t>(item["flags"].GetUint()) : +PolyFlags::Walk;
			area.selectable = item.HasMember("selectable") && item["selectable"].IsBool() ? item["selectable"].GetBool() : true;
			area.valid = true;
			navMesh.UpdateArea(area);
			++count;
		}
		return count;
	}

	template <typename Writer>
	void WriteConnection(Writer& w, const OffMeshConnection& c)
	{
		w.StartObject();
		w.Key("name"); w.String(c.name.c_str());
		w.Key("start"); WriteVec(w, c.start);
		w.Key("end"); WriteVec(w, c.end);
		w.Key("bidirectional"); w.Bool(c.bidirectional);
		w.Key("area"); w.Uint(c.areaType);
		w.Key("type"); w.Uint(static_cast<unsigned>(c.type));
		w.EndObject();
	}

	template <typename Writer>
	void WriteVolume(Writer& w, const ConvexVolume& v)
	{
		w.StartObject();
		w.Key("name"); w.String(v.name.c_str());
		w.Key("verts");
		w.StartArray();
		for (const auto& p : v.verts)
			WriteVec(w, p);
		w.EndArray();
		w.Key("hmin"); w.Double(v.hmin);
		w.Key("hmax"); w.Double(v.hmax);
		w.Key("area"); w.Uint(v.areaType);
		w.EndObject();
	}

	template <typename Writer>
	void WriteArea(Writer& w, const PolyAreaType& a)
	{
		w.StartObject();
		w.Key("id"); w.Uint(a.id);
		w.Key("name"); w.String(a.name.c_str());
		w.Key("cost"); w.Double(a.cost);
		w.Key("flags"); w.Uint(a.flags);
		w.Key("color"); w.Uint(a.color);
		w.Key("selectable"); w.Bool(a.selectable);
		w.EndObject();
	}

	// Copies links, volumes and area types. IDs are assigned fresh, so diff matches by position.
	void CarryFixes(const NavMesh& from, NavMesh& to, std::vector<uint32_t>& connections, std::vector<uint32_t>& volumes,
		int& areas)
	{
		for (const PolyAreaType* area : from.GetPolyAreas())
		{
			to.UpdateArea(*area);
			++areas;
		}
		for (const auto& c : from.GetConnections())
		{
			auto copy = std::make_unique<OffMeshConnection>(*c);
			connections.push_back(to.AddConnection(std::move(copy))->id);
		}
		for (const auto& v : from.GetConvexVolumes())
			volumes.push_back(to.AddConvexVolume(v->verts, v->name, v->hmin, v->hmax, v->areaType)->id);
	}

	//------------------------------------------------------------------------
	// Automatic drop links. They are only proposed: build writes them to a review file and never
	// adds them. A candidate needs an open poly edge, no floor just past it, a ground poly below
	// within the drop range, and clear lines both outward and down.

	struct DropLink
	{
		glm::vec3 start;
		glm::vec3 end;
		float drop;
	};

	std::vector<DropLink> FindDropLinks(NavMesh& navMesh, InputGeom& geom, float maxDrop, float spacing)
	{
		std::vector<DropLink> result;
		const dtNavMesh* mesh = navMesh.GetNavMesh().get();
		auto query = navMesh.GetNavMeshQuery();
		if (!mesh || !query)
			return result;
		const dtQueryFilter filter = MakeFilter(navMesh);
		const NavMeshConfig& config = navMesh.GetNavMeshConfig();
		const float minDrop = config.agentMaxClimb + 0.5f;
		if (maxDrop <= minDrop)
			return result;
		std::set<std::pair<int, int>> used;

		ForEachPoly(mesh, [&](dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) {
			if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION || poly->getArea() != static_cast<uint8_t>(PolyArea::Ground))
				return;
			const auto centre = PolyCentre(tile, poly);
			for (int j = 0; j < poly->vertCount; ++j)
			{
				bool linked = false;
				for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = tile->links[l].next)
					if (tile->links[l].edge == j) { linked = true; break; }
				if (linked)
					continue;

				const float* va = &tile->verts[poly->verts[j] * 3];
				const float* vb = &tile->verts[poly->verts[(j + 1) % poly->vertCount] * 3];
				const glm::vec3 a(va[0], va[1], va[2]), b(vb[0], vb[1], vb[2]);
				const glm::vec3 mid = (a + b) * 0.5f;
				glm::vec3 d = b - a;
				d.y = 0;
				const float len = glm::length(d);
				if (len < 1.0f)
					continue;
				glm::vec3 n(d.z / len, 0, -d.x / len);
				if (glm::dot(n, mid - glm::vec3(centre[0], centre[1], centre[2])) < 0)
					n = -n;

				const std::pair<int, int> key(static_cast<int>(std::floor(mid.x / spacing)), static_cast<int>(std::floor(mid.z / spacing)));
				if (used.count(key))
					continue;

				const glm::vec3 start = mid - n * 0.5f;
				const glm::vec3 out = mid + n * (config.agentRadius + 1.0f);

				// Floor at the same level just outside means this is not a ledge.
				const float sameCentre[3] = { out.x, mid.y, out.z };
				const float sameExt[3] = { 1.0f, minDrop * 0.5f, 1.0f };
				dtPolyRef sameRef = 0;
				float sameHit[3];
				query->findNearestPoly(sameCentre, sameExt, &filter, &sameRef, sameHit);
				if (sameRef)
					continue;

				const float landCentre[3] = { out.x, mid.y - (minDrop + maxDrop) * 0.5f, out.z };
				const float landExt[3] = { 1.0f, (maxDrop - minDrop) * 0.5f, 1.0f };
				dtPolyRef landRef = 0;
				float land[3];
				query->findNearestPoly(landCentre, landExt, &filter, &landRef, land);
				if (!landRef || landRef == ref)
					continue;
				const float drop = mid.y - land[1];
				if (drop < minDrop || drop > maxDrop)
					continue;
				const dtMeshTile* landTile = nullptr;
				const dtPoly* landPoly = nullptr;
				mesh->getTileAndPolyByRefUnsafe(landRef, &landTile, &landPoly);
				if (landPoly->getArea() != static_cast<uint8_t>(PolyArea::Ground))
					continue;

				float t = 0;
				float s1[3] = { start.x, start.y + 1.5f, start.z }, e1[3] = { out.x, mid.y + 1.5f, out.z };
				if (geom.raycastMesh(s1, e1, t))
					continue;
				float s2[3] = { out.x, mid.y + 1.5f, out.z }, e2[3] = { land[0], land[1] + 0.5f, land[2] };
				if (geom.raycastMesh(s2, e2, t))
					continue;

				used.insert(key);
				result.push_back({ start, glm::vec3(land[0], land[1], land[2]), drop });
			}
		});
		return result;
	}

	//------------------------------------------------------------------------
	// Scheduler cap for --threads. The build already runs tiles on the concurrency runtime.

	class SchedulerScope
	{
	public:
		explicit SchedulerScope(unsigned int threads)
		{
			if (threads > 0)
			{
				concurrency::CurrentScheduler::Create(concurrency::SchedulerPolicy(2,
					concurrency::MinConcurrency, 1u, concurrency::MaxConcurrency, threads));
				m_active = true;
			}
		}
		~SchedulerScope()
		{
			if (m_active)
				concurrency::CurrentScheduler::Detach();
		}
		SchedulerScope(const SchedulerScope&) = delete;
		SchedulerScope& operator=(const SchedulerScope&) = delete;

	private:
		bool m_active = false;
	};

	//------------------------------------------------------------------------
	// Commands

	int ValidateMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path meshPath = PathOption(options, "--mesh");
		auto navMesh = LoadMesh(zone, meshPath);
		const int tileCount = CountTiles(navMesh->GetNavMesh().get());
		if (tileCount == 0)
			throw CliError(5, "empty_mesh", "Mesh loaded successfully but contains no tiles.");
		if (!navMesh->GetNavMeshQuery())
			throw CliError(5, "invalid_mesh", "Mesh loaded but its navigation query could not be initialized.");

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("validate");
		w.Key("zone"); w.String(zone.c_str());
		w.Key("mesh"); w.String(meshPath.string().c_str());
		w.Key("tiles"); w.Int(tileCount);
		w.Key("polys"); w.Int(CountPolys(navMesh->GetNavMesh().get()));
		{
			double walkableArea = 0;
			ForEachPoly(navMesh->GetNavMesh().get(), [&](dtPolyRef, const dtMeshTile* tile, const dtPoly* poly) {
				if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION && !(poly->flags & +PolyFlags::Disabled))
					walkableArea += PolyAreaXZ(tile, poly);
			});
			w.Key("walkableArea"); w.Double(std::round(walkableArea));
		}
		w.Key("headerVersion"); w.Int(navMesh->GetHeaderVersion());
		w.Key("connections"); w.Uint64(navMesh->GetConnectionCount());
		w.Key("volumes"); w.Uint64(navMesh->GetConvexVolumeCount());
		w.EndObject();
		Emit(buffer);
		return 0;
	}

	int BuildMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path eqPath = PathOption(options, "--eq-path");
		const fs::path stageRoot = PathOption(options, "--stage-root");
		const fs::path meshRoot = fs::absolute(options.count("--mesh-root")
			? options.at("--mesh-root") : eqPath.string()).lexically_normal();
		const bool regionBuild = options.count("--base") > 0;

		if (regionBuild && (options.count("--preset") || options.count("--settings-from") || HasSettingFlag(options)))
			throw CliError(2, "conflict", "--base takes its settings from the base mesh; drop the settings options.");
		if (regionBuild && options.count("--carry-from"))
			throw CliError(2, "conflict", "--base already keeps the base mesh's fixes; drop --carry-from.");
		if (options.count("--tiles") && options.count("--box"))
			throw CliError(2, "conflict", "--box and --tiles are the same option; give one.");
		if ((options.count("--tiles") || options.count("--box")) && !regionBuild)
			throw CliError(2, "conflict", "--tiles needs --base.");
		if (options.count("--preset") && options.count("--settings-from"))
			throw CliError(2, "conflict", "Use --preset or --settings-from, not both.");
		if ((options.count("--max-drop") || options.count("--drop-spacing")) && !options.count("--auto-drops"))
			throw CliError(2, "conflict", "--max-drop and --drop-spacing need --auto-drops.");

		std::error_code error;
		if (!fs::is_directory(eqPath, error))
			throw CliError(3, "missing_eq_path", "EverQuest geometry directory not found: " + eqPath.string());
		if (!fs::is_directory(meshRoot, error))
			throw CliError(3, "missing_mesh_root", "Mesh metadata root not found: " + meshRoot.string());
		if (stageRoot.string().size() >= MAX_PATH - 16)
			throw CliError(2, "path_too_long", "Stage root is too long for this MQ2Nav build.");

		// A stage root inside a MacroQuest install (its resources folder, or the install itself) is
		// where MQ2Nav reads live meshes. Staging there would put a new zone live unreviewed.
		if (!options.count("--allow-live"))
		{
			const fs::path canonical = fs::weakly_canonical(stageRoot, error);
			for (fs::path dir = canonical; !dir.empty(); dir = dir.parent_path())
			{
				if (fs::exists(dir / "MacroQuest.exe", error) || fs::exists(dir / "MQ2Main.dll", error))
					throw CliError(2, "live_folder", "Stage root is inside a MacroQuest install (" + dir.string()
						+ "), where MQ2Nav reads live meshes. Stage elsewhere, or pass --allow-live.");
				if (dir == dir.parent_path())
					break;
			}
		}

		const fs::path outputDirectory = stageRoot / "MQ2Nav";
		const fs::path outputPath = outputDirectory / (zone + ".navmesh");
		if (fs::exists(outputPath, error))
			throw CliError(4, "output_exists", "Refusing to overwrite staged mesh: " + outputPath.string());
		if (error)
			throw CliError(4, "output_error", "Could not inspect staged output path: " + error.message());

		fs::path dropsPath;
		if (options.count("--auto-drops"))
		{
			dropsPath = PathOption(options, "--auto-drops");
			if (fs::exists(dropsPath, error))
				throw CliError(4, "output_exists", "Refusing to overwrite: " + dropsPath.string());
		}
		const float maxDrop = NumberOption(options, "--max-drop", 20.0f, 1.0f, 200.0f);
		const float dropSpacing = NumberOption(options, "--drop-spacing", 6.0f, 1.0f, 100.0f);
		const unsigned int threads = static_cast<unsigned int>(NumberOption(options, "--threads", 0.0f, 1.0f, 256.0f));
		int headerVersion = static_cast<int>(NavMeshHeaderVersion::Latest);
		if (options.count("--mesh-version"))
		{
			headerVersion = static_cast<int>(NumberOption(options, "--mesh-version", 5.0f, 4.0f, 5.0f));
			if (headerVersion != 4 && headerVersion != 5)
				throw CliError(2, "bad_option", "--mesh-version must be 4 or 5.");
		}

		fs::create_directories(outputDirectory);

		auto navMesh = std::make_shared<NavMesh>(outputDirectory.string(), zone);
		ConsoleRecastContext context;
		auto geometry = std::make_unique<InputGeom>(zone, eqPath.string(), meshRoot.string());
		auto geometryLoader = std::make_unique<MapGeometryLoader>(zone, eqPath.string(), meshRoot.string());
		if (!options.count("--no-max-extents"))
		{
			const auto extents = MaxZoneExtents.find(zone);
			if (extents != MaxZoneExtents.end())
				geometryLoader->SetMaxExtents(extents->second);
		}

		spdlog::info("Loading geometry for zone {}.", zone);
		if (!geometry->loadGeometry(std::move(geometryLoader), &context))
			throw CliError(3, "geometry_load_failed", "Could not load usable zone geometry for: " + zone);

		NavMeshTool navMeshTool(navMesh);
		navMeshTool.setContext(&context);
		navMeshTool.setOutputPath(stageRoot.string().c_str());
		navMeshTool.handleGeometryChanged(geometry.get());
		navMeshTool.SetProtectWalkways(options.count("--walkways") > 0);

		// Settings: preset or another mesh first, then single flags on top.
		std::string settingsSource = "mq2nav";
		if (regionBuild)
		{
			const fs::path basePath = PathOption(options, "--base");
			if (navMesh->LoadNavMeshFile(basePath.string()) != NavMesh::LoadResult::Success || !navMesh->GetNavMesh())
				throw CliError(5, "invalid_mesh", "Base mesh could not be loaded: " + basePath.string());
			navMeshTool.GetConfig() = navMesh->GetNavMeshConfig();
			navMeshTool.UpdateTileSizes();
			headerVersion = options.count("--mesh-version") ? headerVersion : navMesh->GetHeaderVersion();
			settingsSource = "base";

		}
		else
		{
			NavMeshConfig config = navMeshTool.GetConfig();
			if (options.count("--preset"))
			{
				config = PresetConfig(options.at("--preset"));
				settingsSource = options.at("--preset");
			}
			if (options.count("--settings-from"))
			{
				config = LoadMesh(zone, PathOption(options, "--settings-from"))->GetNavMeshConfig();
				settingsSource = "mesh";
			}
			ApplySettingFlags(config, options);
			navMeshTool.GetConfig() = config;
			navMesh->GetNavMeshConfig() = config;
		}
		{
			const NavMeshConfig& config = navMeshTool.GetConfig();
			spdlog::info("Settings ({}): tile {} cell {} cellHeight {} height {} radius {} climb {} slope {}", settingsSource,
				config.tileSize, config.cellSize, config.cellHeight, config.agentHeight,
				config.agentRadius, config.agentMaxClimb, config.agentMaxSlope);
		}

		// Fixes.
		std::vector<uint32_t> carriedConnections, carriedVolumes, newConnections, newVolumes;
		int carriedAreas = 0, newAreas = 0;
		if (options.count("--carry-from"))
		{
			auto from = LoadMesh(zone, PathOption(options, "--carry-from"));
			CarryFixes(*from, *navMesh, carriedConnections, carriedVolumes, carriedAreas);
			if (!options.count("--mesh-version"))
				headerVersion = from->GetHeaderVersion();
			spdlog::info("Carried {} links, {} volumes and {} area types.", carriedConnections.size(),
				carriedVolumes.size(), carriedAreas);
		}
		if (options.count("--fixes"))
		{
			const auto document = ReadJsonFile(options.at("--fixes"), "fixes");
			if (!document.IsObject())
				throw CliError(2, "bad_fixes", "A fixes file is a JSON object, as export writes it.");
			if (document.HasMember("areas"))
				newAreas += AddAreas(document["areas"], *navMesh);
			if (document.HasMember("connections"))
				AddConnections(document["connections"], *navMesh, newConnections);
			if (document.HasMember("volumes"))
				AddVolumes(document["volumes"], *navMesh, newVolumes);
		}
		if (options.count("--connections"))
			AddConnections(ReadJsonFile(options.at("--connections"), "connections"), *navMesh, newConnections);
		if (options.count("--volumes"))
			AddVolumes(ReadJsonFile(options.at("--volumes"), "volumes"), *navMesh, newVolumes);

		SchedulerScope scheduler(threads);
		std::vector<dtTileRef> rebuilt;
		if (regionBuild)
		{
			std::set<dtTileRef> tiles;
			std::set<std::pair<int, int>> emptyCells;
			const dtNavMesh* mesh = navMesh->GetNavMesh().get();
			const dtNavMeshParams* params = mesh->getParams();
			const glm::vec3& boundsMax = navMesh->GetNavMeshBoundsMax();
			const int maxTx = static_cast<int>(std::floor((boundsMax.x - params->orig[0]) / params->tileWidth));
			const int maxTy = static_cast<int>(std::floor((boundsMax.z - params->orig[2]) / params->tileHeight));

			// Every grid cell a box touches: existing tiles are rebuilt, and cells with no tile in the
			// base are refused, because building them would add mesh nobody reviewed.
			const auto addBox = [&](float minX, float maxX, float minZ, float maxZ) {
				const int tx0 = (std::max)(0, static_cast<int>(std::floor((minX - params->orig[0]) / params->tileWidth)));
				const int tx1 = (std::min)(maxTx, static_cast<int>(std::floor((maxX - params->orig[0]) / params->tileWidth)));
				const int ty0 = (std::max)(0, static_cast<int>(std::floor((minZ - params->orig[2]) / params->tileHeight)));
				const int ty1 = (std::min)(maxTy, static_cast<int>(std::floor((maxZ - params->orig[2]) / params->tileHeight)));
				for (int ty = ty0; ty <= ty1; ++ty)
				{
					for (int tx = tx0; tx <= tx1; ++tx)
					{
						const dtMeshTile* tile = mesh->getTileAt(tx, ty, 0);
						if (tile && tile->header)
							tiles.insert(mesh->getTileRef(tile));
						else
							emptyCells.emplace(tx, ty);
					}
				}
			};

			if (options.count("--tiles") || options.count("--box"))
			{
				const std::string region = options.count("--box") ? options.at("--box") : options.at("--tiles");
				const size_t colon = region.find(':');
				if (colon == std::string::npos)
					throw CliError(2, "bad_tiles", "--box must be \"Y1,X1:Y2,X2\" in /loc order.");
				const glm::vec3 a = ParseLoc(region.substr(0, colon) + ",0", "--tiles");
				const glm::vec3 b = ParseLoc(region.substr(colon + 1) + ",0", "--tiles");
				addBox((std::min)(a.x, b.x), (std::max)(a.x, b.x), (std::min)(a.z, b.z), (std::max)(a.z, b.z));
			}
			for (uint32_t id : newConnections)
			{
				const OffMeshConnection* c = navMesh->GetConnectionById(id);
				addBox(c->start.x, c->start.x, c->start.z, c->start.z);
				addBox(c->end.x, c->end.x, c->end.z, c->end.z);
				for (dtTileRef ref : navMesh->GetTilesIntersectingConnection(id))
					tiles.insert(ref);
			}
			for (uint32_t id : newVolumes)
			{
				const ConvexVolume* v = navMesh->GetConvexVolumeById(id);
				float minX = FLT_MAX, maxX = -FLT_MAX, minZ = FLT_MAX, maxZ = -FLT_MAX;
				for (const auto& p : v->verts)
				{
					minX = (std::min)(minX, p.x); maxX = (std::max)(maxX, p.x);
					minZ = (std::min)(minZ, p.z); maxZ = (std::max)(maxZ, p.z);
				}
				addBox(minX, maxX, minZ, maxZ);
			}
			if (!emptyCells.empty())
			{
				std::string list;
				for (const auto& [tx, ty] : emptyCells)
				{
					const float x0 = params->orig[0] + tx * params->tileWidth;
					const float z0 = params->orig[2] + ty * params->tileHeight;
					list += fmt::format("{}tile {},{} (/loc Y {:.0f} to {:.0f}, X {:.0f} to {:.0f})", list.empty() ? "" : "; ",
						tx, ty, z0, z0 + params->tileHeight, x0, x0 + params->tileWidth);
				}
				throw CliError(5, "empty_tiles", "The region covers grid cells the base mesh has no tile for, and a region "
					"rebuild only rebuilds existing tiles. Do a full build instead. Empty cells: " + list);
			}
			if (tiles.empty())
				throw CliError(2, "no_tiles", "Nothing to rebuild: give --box, or links or volumes that touch existing tiles.");
			rebuilt.assign(tiles.begin(), tiles.end());
			spdlog::info("Rebuilding {} tiles of the base mesh.", rebuilt.size());
			navMeshTool.RebuildTiles(rebuilt);
		}
		else
		{
			navMeshTool.UpdateTileSizes();
			spdlog::info("Building zone {} synchronously.", zone);
			if (!navMeshTool.handleBuild(false, false))
				throw CliError(4, "build_failed", "MQ2Nav failed to initialize the navmesh build.");
		}

		const int tileCount = CountTiles(navMesh->GetNavMesh().get());
		if (tileCount == 0)
			throw CliError(4, "empty_build", "Build completed without producing any navmesh tiles.");

		// Links whose ends are no longer on the mesh, within 2 units across and 4 up or down.
		struct Unanchored { const OffMeshConnection* c; bool start; bool end; };
		std::vector<Unanchored> unanchored;
		{
			auto query = navMesh->GetNavMeshQuery();
			const dtQueryFilter filter = MakeFilter(*navMesh);
			const float ext[3] = { 2.f, 4.f, 2.f };
			for (const auto& c : navMesh->GetConnections())
			{
				dtPolyRef s = 0, e = 0;
				float hit[3];
				query->findNearestPoly(glm::value_ptr(c->start), ext, &filter, &s, hit);
				query->findNearestPoly(glm::value_ptr(c->end), ext, &filter, &e, hit);
				if (!s || !e)
					unanchored.push_back({ c.get(), s != 0, e != 0 });
			}
		}

		std::vector<DropLink> drops;
		if (!dropsPath.empty())
		{
			drops = FindDropLinks(*navMesh, *geometry, maxDrop, dropSpacing);
			rapidjson::StringBuffer dropBuffer;
			rapidjson::PrettyWriter<rapidjson::StringBuffer> dw(dropBuffer);
			dw.StartArray();
			for (size_t i = 0; i < drops.size(); ++i)
			{
				dw.StartObject();
				dw.Key("name"); dw.String(fmt::format("auto-drop {}", i + 1).c_str());
				dw.Key("start"); WriteVec(dw, drops[i].start);
				dw.Key("end"); WriteVec(dw, drops[i].end);
				dw.Key("bidirectional"); dw.Bool(false);
				dw.Key("drop"); dw.Double(Round1(drops[i].drop));
				dw.Key("startLoc"); WriteLoc(dw, glm::value_ptr(drops[i].start));
				dw.EndObject();
			}
			dw.EndArray();
			WriteNewFile(dropsPath, std::string(dropBuffer.GetString()) + "\n");
		}

		const fs::path temporaryPath = outputPath.string() + ".building";
		if (fs::exists(temporaryPath, error))
			throw CliError(4, "temporary_exists", "Refusing to overwrite temporary build file: " + temporaryPath.string());

		if (!navMesh->SaveNavMeshFile(temporaryPath.string(), static_cast<NavMeshHeaderVersion>(headerVersion)))
		{
			fs::remove(temporaryPath, error);
			throw CliError(4, "save_failed", "Could not save the staged mesh.");
		}

		NavMesh validation("", zone);
		if (validation.LoadNavMeshFile(temporaryPath.string()) != NavMesh::LoadResult::Success
			|| CountTiles(validation.GetNavMesh().get()) != tileCount
			|| !validation.GetNavMeshQuery())
		{
			fs::remove(temporaryPath, error);
			throw CliError(5, "staged_validation_failed", "The staged mesh did not pass MQ2Nav load/query validation.");
		}

		fs::rename(temporaryPath, outputPath);

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("build");
		w.Key("zone"); w.String(zone.c_str());
		w.Key("mesh"); w.String(outputPath.string().c_str());
		w.Key("tiles"); w.Int(tileCount);
		w.Key("polys"); w.Int(CountPolys(navMesh->GetNavMesh().get()));
		w.Key("validation"); w.String("passed");
		w.Key("headerVersion"); w.Int(headerVersion);
		w.Key("settingsSource"); w.String(settingsSource.c_str());
		w.Key("generator"); w.String("MeshGenerator headless CLI 2026-11-25");
		w.Key("inputs");
		w.StartObject();
		for (const char* key : { "--base", "--settings-from", "--carry-from", "--fixes", "--connections", "--volumes" })
		{
			if (!options.count(key))
				continue;
			const fs::path input = PathOption(options, key);
			w.Key(key + 2);
			w.StartObject();
			w.Key("file"); w.String(input.string().c_str());
			w.Key("sha256"); w.String(FileSha256(input).c_str());
			w.EndObject();
		}
		w.EndObject();
		w.Key("settings"); WriteSettings(w, navMeshTool.GetConfig());
		w.Key("walkways"); w.Bool(options.count("--walkways") > 0);
		w.Key("carried");
		w.StartObject();
		w.Key("connections"); w.Uint64(carriedConnections.size());
		w.Key("volumes"); w.Uint64(carriedVolumes.size());
		w.Key("areas"); w.Int(carriedAreas);
		w.EndObject();
		w.Key("added");
		w.StartObject();
		w.Key("connections"); w.Uint64(newConnections.size());
		w.Key("volumes"); w.Uint64(newVolumes.size());
		w.Key("areas"); w.Int(newAreas);
		w.EndObject();
		w.Key("totalConnections"); w.Uint64(navMesh->GetConnectionCount());
		w.Key("totalVolumes"); w.Uint64(navMesh->GetConvexVolumeCount());
		if (regionBuild)
		{
			w.Key("rebuiltTiles"); w.Uint64(rebuilt.size());
		}
		w.Key("unanchoredLinks");
		w.StartArray();
		for (const auto& u : unanchored)
		{
			w.StartObject();
			w.Key("name"); w.String(u.c->name.c_str());
			w.Key("startLoc"); WriteLoc(w, glm::value_ptr(u.c->start));
			w.Key("endLoc"); WriteLoc(w, glm::value_ptr(u.c->end));
			w.Key("startOnMesh"); w.Bool(u.start);
			w.Key("endOnMesh"); w.Bool(u.end);
			w.EndObject();
		}
		w.EndArray();
		if (!dropsPath.empty())
		{
			w.Key("autoDrops");
			w.StartObject();
			w.Key("file"); w.String(dropsPath.string().c_str());
			w.Key("candidates"); w.Uint64(drops.size());
			w.Key("added"); w.Bool(false);
			w.EndObject();
		}
		w.EndObject();
		Emit(buffer);
		return 0;
	}

	int ExportFixes(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path meshPath = PathOption(options, "--mesh");
		const fs::path outPath = PathOption(options, "--out");
		auto navMesh = LoadMesh(zone, meshPath);

		rapidjson::StringBuffer fileBuffer;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> fw(fileBuffer);
		fw.StartObject();
		fw.Key("schemaVersion"); fw.Int(1);
		fw.Key("zone"); fw.String(zone.c_str());
		fw.Key("sourceMesh"); fw.String(meshPath.string().c_str());
		fw.Key("headerVersion"); fw.Int(navMesh->GetHeaderVersion());
		fw.Key("settings"); WriteSettings(fw, navMesh->GetNavMeshConfig());
		fw.Key("connections");
		fw.StartArray();
		for (const auto& c : navMesh->GetConnections())
			WriteConnection(fw, *c);
		fw.EndArray();
		fw.Key("volumes");
		fw.StartArray();
		for (const auto& v : navMesh->GetConvexVolumes())
			WriteVolume(fw, *v);
		fw.EndArray();
		fw.Key("areas");
		fw.StartArray();
		std::vector<const PolyAreaType*> areas(navMesh->GetPolyAreas().begin(), navMesh->GetPolyAreas().end());
		std::sort(areas.begin(), areas.end(), [](const PolyAreaType* a, const PolyAreaType* b) { return a->id < b->id; });
		for (const PolyAreaType* a : areas)
			WriteArea(fw, *a);
		fw.EndArray();
		fw.EndObject();
		WriteNewFile(outPath, std::string(fileBuffer.GetString()) + "\n");

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("export");
		w.Key("out"); w.String(outPath.string().c_str());
		w.Key("connections"); w.Uint64(navMesh->GetConnectionCount());
		w.Key("volumes"); w.Uint64(navMesh->GetConvexVolumeCount());
		w.Key("areas"); w.Uint64(areas.size());
		w.EndObject();
		Emit(buffer);
		return 0;
	}

	int DiffMeshes(const Options& options)
	{
		const std::string zone = GetZone(options);
		auto oldMesh = LoadMesh(zone, PathOption(options, "--old"));
		auto newMesh = LoadMesh(zone, PathOption(options, "--new"));
		const float tol = NumberOption(options, "--tolerance", 0.5f, 0.0f, 50.0f);
		const auto closeTo = [tol](const glm::vec3& a, const glm::vec3& b) { return glm::distance(a, b) <= tol; };

		const auto& oldLinks = oldMesh->GetConnections();
		const auto& newLinks = newMesh->GetConnections();
		std::vector<bool> linkUsed(newLinks.size(), false);
		std::vector<const OffMeshConnection*> lostLinks, changedLinks;
		for (const auto& o : oldLinks)
		{
			bool found = false;
			for (size_t i = 0; i < newLinks.size() && !found; ++i)
			{
				const auto& n = newLinks[i];
				if (linkUsed[i])
					continue;
				const bool same = closeTo(o->start, n->start) && closeTo(o->end, n->end);
				const bool swapped = o->bidirectional && n->bidirectional && closeTo(o->start, n->end) && closeTo(o->end, n->start);
				if (same || swapped)
				{
					linkUsed[i] = found = true;
					if (o->bidirectional != n->bidirectional || o->areaType != n->areaType || o->name != n->name
						|| o->type != n->type)
						changedLinks.push_back(o.get());
				}
			}
			if (!found)
				lostLinks.push_back(o.get());
		}

		const auto& oldVols = oldMesh->GetConvexVolumes();
		const auto& newVols = newMesh->GetConvexVolumes();
		std::vector<bool> volUsed(newVols.size(), false);
		std::vector<const ConvexVolume*> lostVols, changedVols;
		for (const auto& o : oldVols)
		{
			bool found = false;
			for (size_t i = 0; i < newVols.size() && !found; ++i)
			{
				const auto& n = newVols[i];
				if (volUsed[i] || n->verts.size() != o->verts.size()
					|| std::fabs(n->hmin - o->hmin) > tol || std::fabs(n->hmax - o->hmax) > tol)
					continue;
				// Same outline whatever vertex the list starts at, in either winding.
				const size_t count = o->verts.size();
				bool same = false;
				for (size_t shift = 0; shift < count && !same; ++shift)
				{
					for (int dir = 1; dir >= -1 && !same; dir -= 2)
					{
						bool all = true;
						for (size_t v = 0; v < count && all; ++v)
						{
							const size_t k = (shift + count + dir * static_cast<long long>(v) % static_cast<long long>(count)) % count;
							all = closeTo(o->verts[v], n->verts[k]);
						}
						same = all;
					}
				}
				if (same)
				{
					volUsed[i] = found = true;
					if (o->areaType != n->areaType || o->name != n->name)
						changedVols.push_back(o.get());
				}
			}
			if (!found)
				lostVols.push_back(o.get());
		}

		std::map<int, const PolyAreaType*> oldAreas, newAreas;
		for (const PolyAreaType* a : oldMesh->GetPolyAreas()) oldAreas[a->id] = a;
		for (const PolyAreaType* a : newMesh->GetPolyAreas()) newAreas[a->id] = a;
		std::vector<int> lostAreas, addedAreas, changedAreas;
		for (const auto& [id, a] : oldAreas)
		{
			const auto it = newAreas.find(id);
			if (it == newAreas.end())
				lostAreas.push_back(id);
			else if (!(*a == *it->second))
				changedAreas.push_back(id);
		}
		for (const auto& [id, a] : newAreas)
			if (!oldAreas.count(id))
				addedAreas.push_back(id);

		const size_t lost = lostLinks.size() + lostVols.size() + lostAreas.size();

		// Tile geometry: a tile changed when its polygon count or vertices differ. Hand-built tiles
		// that a rebuild cannot reproduce show up here even when no link or volume was lost.
		struct TileKey { int x, y, layer; bool operator<(const TileKey& o) const { return std::tie(x, y, layer) < std::tie(o.x, o.y, o.layer); } };
		const auto tileSignatures = [](const dtNavMesh* mesh) {
			std::map<TileKey, std::pair<uint64_t, std::array<float, 4>>> out;
			for (int i = 0; i < mesh->getMaxTiles(); ++i)
			{
				const dtMeshTile* tile = mesh->getTile(i);
				if (!tile || !tile->header)
					continue;
				uint64_t h = 1469598103934665603ull;
				const auto mix = [&h](const void* data, size_t size) {
					const unsigned char* bytes = static_cast<const unsigned char*>(data);
					for (size_t b = 0; b < size; ++b) { h ^= bytes[b]; h *= 1099511628211ull; }
				};
				mix(&tile->header->polyCount, sizeof(int));
				for (int v = 0; v < tile->header->vertCount * 3; ++v)
				{
					const int rounded = static_cast<int>(std::lround(tile->verts[v] * 10.0f));
					mix(&rounded, sizeof(rounded));
				}
				out[{ tile->header->x, tile->header->y, tile->header->layer }] = { h,
					{ tile->header->bmin[0], tile->header->bmin[2], tile->header->bmax[0], tile->header->bmax[2] } };
			}
			return out;
		};
		const auto oldTiles = tileSignatures(oldMesh->GetNavMesh().get());
		const auto newTiles = tileSignatures(newMesh->GetNavMesh().get());
		std::vector<std::pair<TileKey, std::array<float, 4>>> changedTiles;
		size_t tilesOnlyOld = 0, tilesOnlyNew = 0;
		for (const auto& [key, sig] : oldTiles)
		{
			const auto it = newTiles.find(key);
			if (it == newTiles.end())
				++tilesOnlyOld;
			else if (it->second.first != sig.first)
				changedTiles.emplace_back(key, sig.second);
		}
		for (const auto& [key, sig] : newTiles)
			if (!oldTiles.count(key))
				++tilesOnlyNew;
		bool settingsDiffer = false;
		for (const Setting& s : kSettings)
			if (std::fabs(oldMesh->GetNavMeshConfig().*s.field - newMesh->GetNavMeshConfig().*s.field) > 1e-4f)
				settingsDiffer = true;
		const bool strict = options.count("--strict") > 0;

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("diff");
		w.Key("lost"); w.Uint64(lost);
		w.Key("tileGeometry");
		w.StartObject();
		w.Key("changed"); w.Uint64(changedTiles.size());
		w.Key("onlyOld"); w.Uint64(tilesOnlyOld);
		w.Key("onlyNew"); w.Uint64(tilesOnlyNew);
		w.Key("changedTiles");
		w.StartArray();
		for (size_t i = 0; i < changedTiles.size() && i < 50; ++i)
		{
			const auto& [key, box] = changedTiles[i];
			w.StartObject();
			w.Key("tile"); w.StartArray(); w.Int(key.x); w.Int(key.y); w.Int(key.layer); w.EndArray();
			w.Key("locY"); w.StartArray(); w.Double(Round1(box[1])); w.Double(Round1(box[3])); w.EndArray();
			w.Key("locX"); w.StartArray(); w.Double(Round1(box[0])); w.Double(Round1(box[2])); w.EndArray();
			w.EndObject();
		}
		w.EndArray();
		w.EndObject();
		w.Key("tolerance"); w.Double(tol);
		w.Key("tiles");
		w.StartObject();
		w.Key("old"); w.Int(CountTiles(oldMesh->GetNavMesh().get()));
		w.Key("new"); w.Int(CountTiles(newMesh->GetNavMesh().get()));
		w.EndObject();
		w.Key("polys");
		w.StartObject();
		w.Key("old"); w.Int(CountPolys(oldMesh->GetNavMesh().get()));
		w.Key("new"); w.Int(CountPolys(newMesh->GetNavMesh().get()));
		w.EndObject();
		w.Key("headerVersion");
		w.StartObject();
		w.Key("old"); w.Int(oldMesh->GetHeaderVersion());
		w.Key("new"); w.Int(newMesh->GetHeaderVersion());
		w.EndObject();
		w.Key("settingsChanged");
		w.StartArray();
		for (const Setting& s : kSettings)
		{
			const float a = oldMesh->GetNavMeshConfig().*s.field, b = newMesh->GetNavMeshConfig().*s.field;
			if (std::fabs(a - b) > 1e-4f)
			{
				w.StartObject();
				w.Key("setting"); w.String(s.key);
				w.Key("old"); w.Double(a);
				w.Key("new"); w.Double(b);
				w.EndObject();
			}
		}
		w.EndArray();

		const auto writeLinks = [&w](const char* key, const std::vector<const OffMeshConnection*>& list) {
			w.Key(key);
			w.StartArray();
			for (const auto* c : list)
			{
				w.StartObject();
				w.Key("name"); w.String(c->name.c_str());
				w.Key("startLoc"); WriteLoc(w, glm::value_ptr(c->start));
				w.Key("endLoc"); WriteLoc(w, glm::value_ptr(c->end));
				w.EndObject();
			}
			w.EndArray();
		};
		const auto writeVols = [&w](const char* key, const std::vector<const ConvexVolume*>& list) {
			w.Key(key);
			w.StartArray();
			for (const auto* v : list)
			{
				w.StartObject();
				w.Key("name"); w.String(v->name.c_str());
				w.Key("area"); w.Uint(v->areaType);
				w.Key("firstVertLoc"); WriteLoc(w, glm::value_ptr(v->verts[0]));
				w.EndObject();
			}
			w.EndArray();
		};
		const auto writeIds = [&w](const char* key, const std::vector<int>& ids) {
			w.Key(key);
			w.StartArray();
			for (int id : ids) w.Int(id);
			w.EndArray();
		};

		std::vector<const OffMeshConnection*> addedLinks;
		for (size_t i = 0; i < newLinks.size(); ++i)
			if (!linkUsed[i]) addedLinks.push_back(newLinks[i].get());
		std::vector<const ConvexVolume*> addedVols;
		for (size_t i = 0; i < newVols.size(); ++i)
			if (!volUsed[i]) addedVols.push_back(newVols[i].get());

		w.Key("connections");
		w.StartObject();
		w.Key("old"); w.Uint64(oldLinks.size());
		w.Key("new"); w.Uint64(newLinks.size());
		writeLinks("lost", lostLinks);
		writeLinks("added", addedLinks);
		writeLinks("changed", changedLinks);
		w.EndObject();
		w.Key("volumes");
		w.StartObject();
		w.Key("old"); w.Uint64(oldVols.size());
		w.Key("new"); w.Uint64(newVols.size());
		writeVols("lost", lostVols);
		writeVols("added", addedVols);
		writeVols("changed", changedVols);
		w.EndObject();
		w.Key("areas");
		w.StartObject();
		writeIds("lost", lostAreas);
		writeIds("added", addedAreas);
		writeIds("changed", changedAreas);
		w.EndObject();
		w.EndObject();
		Emit(buffer);
		const bool strictFail = strict && (settingsDiffer || !changedTiles.empty() || tilesOnlyOld > 0 || tilesOnlyNew > 0);
		return (lost > 0 || strictFail) ? 6 : 0;
	}

	template <typename Writer>
	void WritePath(Writer& w, NavMesh& navMesh, const PathResult& path)
	{
		w.Key("status"); w.String(path.status.c_str());
		w.Key("length"); w.Double(Round1(path.length));
		w.Key("links"); w.Int(path.links);
		w.Key("waypoints");
		w.StartArray();
		const dtNavMesh* mesh = navMesh.GetNavMesh().get();
		for (size_t i = 0; i < path.flags.size(); ++i)
		{
			w.StartObject();
			w.Key("loc"); WriteLoc(w, &path.verts[i * 3]);
			w.Key("link"); w.Bool((path.flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0);
			if (path.polys[i])
			{
				const dtMeshTile* tile = nullptr;
				const dtPoly* poly = nullptr;
				if (dtStatusSucceed(mesh->getTileAndPolyByRef(path.polys[i], &tile, &poly)))
				{
					w.Key("area"); w.String(navMesh.GetPolyArea(poly->getArea()).name.c_str());
				}
			}
			w.EndObject();
		}
		w.EndArray();
	}

	int PathCommand(const Options& options)
	{
		const std::string zone = GetZone(options);
		auto navMesh = LoadMesh(zone, PathOption(options, "--mesh"));
		auto query = navMesh->GetNavMeshQuery();
		if (!query)
			throw CliError(5, "invalid_mesh", "No navigation query.");
		float extents[3];
		GetExtents(options, extents);
		const glm::vec3 from = ParseLoc(RequiredOption(options, "--from"), "--from");
		const glm::vec3 to = ParseLoc(RequiredOption(options, "--to"), "--to");
		const dtQueryFilter filter = MakeFilter(*navMesh);
		const PathResult path = PlanPath(*query, filter, extents, from, to);

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("path");
		w.Key("from"); WriteLoc(w, glm::value_ptr(from));
		w.Key("to"); WriteLoc(w, glm::value_ptr(to));
		WritePath(w, *navMesh, path);
		w.EndObject();
		Emit(buffer);
		return path.status == "found" ? 0 : 6;
	}

	int ReachCommand(const Options& options)
	{
		const std::string zone = GetZone(options);
		auto navMesh = LoadMesh(zone, PathOption(options, "--mesh"));
		auto query = navMesh->GetNavMeshQuery();
		float extents[3];
		GetExtents(options, extents);
		const glm::vec3 from = ParseLoc(RequiredOption(options, "--from"), "--from");
		const int minPolys = static_cast<int>(NumberOption(options, "--min-polys", 3.0f, 1.0f, 1e6f));
		const int limit = static_cast<int>(NumberOption(options, "--limit", 50.0f, 1.0f, 1e6f));
		const dtQueryFilter filter = MakeFilter(*navMesh);
		const dtNavMesh* mesh = navMesh->GetNavMesh().get();

		dtPolyRef startRef = 0;
		float hit[3];
		query->findNearestPoly(glm::value_ptr(from), extents, &filter, &startRef, hit);
		if (!startRef)
			throw CliError(5, "start_not_on_mesh", "--from is not on the mesh within the search box.");

		const auto reached = Flood(mesh, filter, startRef);
		struct Island { int polys = 0; float area = 0; std::array<float, 3> sample{}; };
		std::vector<Island> islands;
		std::unordered_set<dtPolyRef> assigned;
		int walkable = 0;
		ForEachPoly(mesh, [&](dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) {
			if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION || !filter.passFilter(ref, tile, poly))
				return;
			++walkable;
			if (reached.count(ref) || assigned.count(ref))
				return;
			const auto part = Flood(mesh, filter, ref, &reached);
			Island island;
			island.sample = PolyCentre(tile, poly);
			for (dtPolyRef r : part)
			{
				if (assigned.count(r))
					continue;
				assigned.insert(r);
				const dtMeshTile* t = nullptr;
				const dtPoly* p = nullptr;
				mesh->getTileAndPolyByRefUnsafe(r, &t, &p);
				if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				++island.polys;
				island.area += PolyAreaXZ(t, p);
			}
			if (island.polys > 0)
				islands.push_back(island);
		});
		std::sort(islands.begin(), islands.end(), [](const Island& a, const Island& b) { return a.area > b.area; });

		int reachedWalkable = 0;
		for (dtPolyRef r : reached)
		{
			const dtMeshTile* t = nullptr;
			const dtPoly* p = nullptr;
			mesh->getTileAndPolyByRefUnsafe(r, &t, &p);
			if (p->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
				++reachedWalkable;
		}

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("reach");
		w.Key("from"); WriteLoc(w, glm::value_ptr(from));
		w.Key("walkablePolys"); w.Int(walkable);
		w.Key("reachedPolys"); w.Int(reachedWalkable);
		w.Key("islandCount"); w.Uint64(islands.size());
		w.Key("islands");
		w.StartArray();
		int shown = 0;
		for (const Island& island : islands)
		{
			if (island.polys < minPolys || shown >= limit)
				continue;
			++shown;
			w.StartObject();
			w.Key("polys"); w.Int(island.polys);
			w.Key("area"); w.Double(Round1(island.area));
			w.Key("sampleLoc"); WriteLoc(w, island.sample.data());
			w.EndObject();
		}
		w.EndArray();
		w.Key("shown"); w.Int(shown);
		w.EndObject();
		Emit(buffer);
		return shown > 0 ? 6 : 0;
	}

	bool PointInVolume(const ConvexVolume& v, const glm::vec3& p)
	{
		if (p.y < v.hmin || p.y > v.hmax || v.verts.size() < 3)
			return false;
		bool inside = false;
		for (size_t i = 0, j = v.verts.size() - 1; i < v.verts.size(); j = i++)
		{
			const glm::vec3& a = v.verts[i];
			const glm::vec3& b = v.verts[j];
			if (((a.z > p.z) != (b.z > p.z)) && (p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x))
				inside = !inside;
		}
		return inside;
	}

	int InspectCommand(const Options& options)
	{
		const std::string zone = GetZone(options);
		auto navMesh = LoadMesh(zone, PathOption(options, "--mesh"));
		auto query = navMesh->GetNavMeshQuery();
		float extents[3];
		GetExtents(options, extents);
		const glm::vec3 at = ParseLoc(RequiredOption(options, "--at"), "--at");
		const float radius = NumberOption(options, "--radius", 30.0f, 0.0f, 10000.0f);
		const dtQueryFilter filter = MakeFilter(*navMesh);
		const dtNavMesh* mesh = navMesh->GetNavMesh().get();

		dtPolyRef ref = 0, wideRef = 0;
		float hit[3] = { 0, 0, 0 }, wideHit[3] = { 0, 0, 0 };
		query->findNearestPoly(glm::value_ptr(at), extents, &filter, &ref, hit);
		const float wide[3] = { 60.f, 60.f, 60.f };
		query->findNearestPoly(glm::value_ptr(at), wide, &filter, &wideRef, wideHit);

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("inspect");
		w.Key("at"); WriteLoc(w, glm::value_ptr(at));
		w.Key("onMesh"); w.Bool(ref != 0);
		w.Key("nearestWithin60");
		if (wideRef) WriteLoc(w, wideHit); else w.Null();
		w.Key("headerVersion"); w.Int(navMesh->GetHeaderVersion());
		w.Key("settings"); WriteSettings(w, navMesh->GetNavMeshConfig());
		if (ref)
		{
			const dtMeshTile* tile = nullptr;
			const dtPoly* poly = nullptr;
			mesh->getTileAndPolyByRefUnsafe(ref, &tile, &poly);
			int linked = 0, open = 0;
			for (int j = 0; j < poly->vertCount; ++j)
			{
				bool has = false;
				for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = tile->links[l].next)
					if (tile->links[l].edge == j) { has = true; break; }
				has ? ++linked : ++open;
			}
			float height = hit[1];
			query->getPolyHeight(ref, hit, &height);
			w.Key("poly");
			w.StartObject();
			w.Key("ref"); w.Uint64(ref);
			w.Key("nearestLoc"); WriteLoc(w, hit);
			w.Key("distance"); w.Double(Round1(dtVdist(hit, glm::value_ptr(at))));
			w.Key("height"); w.Double(Round1(height));
			w.Key("tile");
			w.StartArray();
			w.Int(tile->header->x); w.Int(tile->header->y); w.Int(tile->header->layer);
			w.EndArray();
			w.Key("areaId"); w.Uint(poly->getArea());
			w.Key("area"); w.String(navMesh->GetPolyArea(poly->getArea()).name.c_str());
			w.Key("areaCost"); w.Double(navMesh->GetPolyArea(poly->getArea()).cost);
			w.Key("flags"); w.Uint(poly->flags);
			w.Key("disabled"); w.Bool((poly->flags & +PolyFlags::Disabled) != 0);
			w.Key("verts"); w.Int(poly->vertCount);
			w.Key("linkedEdges"); w.Int(linked);
			w.Key("openEdges"); w.Int(open);
			w.EndObject();
		}
		w.Key("linksNearby");
		w.StartArray();
		for (const auto& c : navMesh->GetConnections())
		{
			const float d = (std::min)(glm::distance(c->start, at), glm::distance(c->end, at));
			if (d > radius)
				continue;
			w.StartObject();
			w.Key("name"); w.String(c->name.c_str());
			w.Key("startLoc"); WriteLoc(w, glm::value_ptr(c->start));
			w.Key("endLoc"); WriteLoc(w, glm::value_ptr(c->end));
			w.Key("bidirectional"); w.Bool(c->bidirectional);
			w.Key("distance"); w.Double(Round1(d));
			w.EndObject();
		}
		w.EndArray();
		w.Key("volumesHere");
		w.StartArray();
		for (const auto& v : navMesh->GetConvexVolumes())
		{
			if (!PointInVolume(*v, at))
				continue;
			w.StartObject();
			w.Key("name"); w.String(v->name.c_str());
			w.Key("areaId"); w.Uint(v->areaType);
			w.Key("area"); w.String(navMesh->GetPolyArea(v->areaType).name.c_str());
			w.EndObject();
		}
		w.EndArray();
		w.EndObject();
		Emit(buffer);
		return 0;
	}

	// Without --routes: whether each point is on the mesh and reachable from the first, and where
	// the break is. With --routes: a regression suite planned the way the plugin plans.
	int ProbeMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path meshPath = PathOption(options, "--mesh");
		if (options.count("--points") == options.count("--routes"))
			throw CliError(2, "invalid_argument", "probe needs --points or --routes (one of them).");

		struct Route
		{
			std::string name;
			std::vector<glm::vec3> points;
			bool expectReach = true;
			float maxLength = 0;
			bool hasEndZ = false;
			float endZ = 0;
			float endTolerance = 5.0f;
		};
		std::vector<Route> routes;
		std::vector<glm::vec3> locs;
		if (options.count("--routes"))
		{
			const auto document = ReadJsonFile(options.at("--routes"), "routes");
			if (!document.IsArray())
				throw CliError(2, "bad_routes", "A routes file is a JSON array.");
			for (const auto& item : document.GetArray())
			{
				Route route;
				if (!item.IsObject() || !item.HasMember("name") || !item["name"].IsString() || !item.HasMember("points"))
					throw CliError(2, "bad_routes", "Each route needs name and points.");
				route.name = item["name"].GetString();
				if (item["points"].IsString())
					route.points = ParseLocList(item["points"].GetString(), "points");
				else if (item["points"].IsArray())
				{
					for (const auto& p : item["points"].GetArray())
					{
						if (!p.IsArray() || p.Size() != 3 || !p[0].IsNumber() || !p[1].IsNumber() || !p[2].IsNumber())
							throw CliError(2, "bad_routes", "Route points are [y,x,z] in /loc order.");
						route.points.emplace_back(p[1].GetFloat(), p[2].GetFloat(), p[0].GetFloat());
					}
				}
				if (route.points.size() < 2)
					throw CliError(2, "bad_routes", "Route " + route.name + " needs at least two points.");
				if (item.HasMember("expect"))
				{
					const std::string expect = item["expect"].IsString() ? item["expect"].GetString() : "";
					if (expect != "reach" && expect != "noreach")
						throw CliError(2, "bad_routes", "expect must be reach or noreach.");
					route.expectReach = expect == "reach";
				}
				if (item.HasMember("maxLength"))
				{
					if (!item["maxLength"].IsNumber() || item["maxLength"].GetFloat() <= 0)
						throw CliError(2, "bad_routes", "maxLength must be a positive number.");
					route.maxLength = item["maxLength"].GetFloat();
				}
				if (item.HasMember("endZ"))
				{
					if (!item["endZ"].IsNumber())
						throw CliError(2, "bad_routes", "endZ must be a number (the height the route should end at, /loc Z).");
					route.hasEndZ = true;
					route.endZ = item["endZ"].GetFloat();
				}
				if (item.HasMember("endTolerance"))
				{
					if (!item["endTolerance"].IsNumber() || item["endTolerance"].GetFloat() <= 0)
						throw CliError(2, "bad_routes", "endTolerance must be a positive number.");
					route.endTolerance = item["endTolerance"].GetFloat();
				}
				routes.push_back(std::move(route));
			}
		}
		else
		{
			locs = ParseLocList(options.at("--points"), "--points");
		}

		auto navMesh = LoadMesh(zone, meshPath);
		auto query = navMesh->GetNavMeshQuery();
		if (!query)
			throw CliError(5, "invalid_mesh", "No navigation query.");
		float extents[3];
		GetExtents(options, extents);
		const dtQueryFilter filter = MakeFilter(*navMesh);

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("probe");
		w.Key("mesh"); w.String(meshPath.string().c_str());
		w.Key("tiles"); w.Int(CountTiles(navMesh->GetNavMesh().get()));
		w.Key("settings"); WriteSettings(w, navMesh->GetNavMeshConfig());

		if (!routes.empty())
		{
			int passed = 0, failed = 0;
			w.Key("routes");
			w.StartArray();
			for (const Route& route : routes)
			{
				bool allFound = true;
				float total = 0;
				bool haveEnd = false;
				float end[3] = { 0, 0, 0 };
				w.StartObject();
				w.Key("name"); w.String(route.name.c_str());
				w.Key("expect"); w.String(route.expectReach ? "reach" : "noreach");
				w.Key("legs");
				w.StartArray();
				for (size_t i = 1; i < route.points.size(); ++i)
				{
					const PathResult leg = PlanPath(*query, filter, extents, route.points[i - 1], route.points[i]);
					allFound = allFound && leg.status == "found";
					total += leg.length;
					if (i + 1 == route.points.size() && leg.status == "found" && leg.verts.size() >= 3)
					{
						dtVcopy(end, &leg.verts[leg.verts.size() - 3]);
						haveEnd = true;
					}
					w.StartObject();
					w.Key("from"); WriteLoc(w, glm::value_ptr(route.points[i - 1]));
					w.Key("to"); WriteLoc(w, glm::value_ptr(route.points[i]));
					w.Key("status"); w.String(leg.status.c_str());
					w.Key("length"); w.Double(Round1(leg.length));
					w.Key("links"); w.Int(leg.links);
					w.EndObject();
				}
				w.EndArray();
				// Where the route really ends, against the target: the search box can snap the end to
				// another floor or a ledge below. The horizontal gap always counts; the height counts
				// when the route gives endZ (best taken from where a character stopped in game).
				const glm::vec3& target = route.points.back();
				const float endGapXZ = haveEnd ? std::hypot(end[0] - target.x, end[2] - target.z) : 0.0f;
				const float endGapZ = haveEnd ? std::fabs(end[1] - (route.hasEndZ ? route.endZ : target.y)) : 0.0f;
				const bool endOk = !haveEnd || (endGapXZ <= route.endTolerance * 2
					&& (!route.hasEndZ || endGapZ <= route.endTolerance));
				const bool pass = route.expectReach
					? (allFound && endOk && (route.maxLength <= 0 || total <= route.maxLength))
					: !allFound;
				pass ? ++passed : ++failed;
				w.Key("found"); w.Bool(allFound);
				w.Key("length"); w.Double(Round1(total));
				if (haveEnd)
				{
					w.Key("endLoc"); WriteLoc(w, end);
					w.Key("endGapXZ"); w.Double(Round1(endGapXZ));
					w.Key("endGapZ"); w.Double(Round1(endGapZ));
					w.Key("endOk"); w.Bool(endOk);
				}
				if (route.maxLength > 0)
				{
					w.Key("maxLength"); w.Double(route.maxLength);
				}
				w.Key("pass"); w.Bool(pass);
				w.EndObject();
			}
			w.EndArray();
			w.Key("passed"); w.Int(passed);
			w.Key("failed"); w.Int(failed);
			w.EndObject();
			Emit(buffer);
			return failed > 0 ? 6 : 0;
		}

		const dtNavMesh* mesh = query->getAttachedNavMesh();
		const float wide[3] = { 60.f, 60.f, 60.f };
		dtPolyRef startRef = 0;
		float startPos[3] = { 0, 0, 0 };
		bool first = true;
		bool gapDone = false;
		dtPolyRef gapFrom = 0;
		std::vector<dtPolyRef> path(4096);

		w.Key("points");
		w.StartArray();
		for (const auto& pos : locs)
		{
			dtPolyRef ref = 0, wideRef = 0;
			float hit[3] = { 0, 0, 0 }, wideHit[3] = { 0, 0, 0 };
			query->findNearestPoly(glm::value_ptr(pos), extents, &filter, &ref, hit);
			query->findNearestPoly(glm::value_ptr(pos), wide, &filter, &wideRef, wideHit);
			if (first) { startRef = ref; dtVcopy(startPos, hit); first = false; }

			// no_start: the first point is off the mesh, so no route can be tested.
			// search_limit: Detour ran out of nodes or path space, which is not proof of a break.
			std::string reach = !startRef ? "no_start" : "not_on_mesh";
			int polys = 0;
			bool haveForward = false, haveBack = false;
			float endForward[3], endBack[3];
			if (ref && startRef)
			{
				const dtStatus status = query->findPath(startRef, ref, startPos, hit, &filter, path.data(), &polys, 4096);
				if (dtStatusFailed(status))
					reach = "failed";
				else if (polys > 0 && path[polys - 1] == ref)
					reach = "reachable";
				else if (dtStatusDetail(status, DT_OUT_OF_NODES) || dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
					reach = "search_limit";
				else if (polys == 0)
					reach = "failed";
				else
					reach = "partial";
				if (reach == "partial")
				{
					haveForward = dtStatusSucceed(query->closestPointOnPoly(path[polys - 1], hit, endForward, nullptr));
					int backPolys = 0;
					haveBack = dtStatusSucceed(query->findPath(ref, startRef, hit, startPos, &filter, path.data(), &backPolys, 4096))
						&& backPolys > 0
						&& dtStatusSucceed(query->closestPointOnPoly(path[backPolys - 1], startPos, endBack, nullptr));
					if (!gapDone)
					{
						gapDone = true;
						gapFrom = ref;
					}
				}
			}
			w.StartObject();
			w.Key("loc"); WriteLoc(w, glm::value_ptr(pos));
			w.Key("onMesh"); w.Bool(ref != 0);
			w.Key("route"); w.String(reach.c_str());
			w.Key("polys"); w.Int(polys);
			w.Key("nearestWithin60");
			if (wideRef) WriteLoc(w, wideHit); else w.Null();
			w.Key("startSideEnd");
			if (haveForward) WriteLoc(w, endForward); else w.Null();
			w.Key("pointSideEnd");
			if (haveBack) WriteLoc(w, endBack); else w.Null();
			w.EndObject();
		}
		w.EndArray();

		w.Key("gap");
		if (gapFrom)
		{
			const auto a = Flood(mesh, filter, startRef), b = Flood(mesh, filter, gapFrom);
			std::vector<std::array<float, 3>> ca, cb;
			for (dtPolyRef r : a) { const dtMeshTile* t; const dtPoly* p; mesh->getTileAndPolyByRefUnsafe(r, &t, &p); ca.push_back(PolyCentre(t, p)); }
			for (dtPolyRef r : b) { const dtMeshTile* t; const dtPoly* p; mesh->getTileAndPolyByRefUnsafe(r, &t, &p); cb.push_back(PolyCentre(t, p)); }
			// Every pair up to about 20 million checks; past that the larger side is sampled.
			const auto& fewer = ca.size() <= cb.size() ? ca : cb;
			const auto& more = ca.size() <= cb.size() ? cb : ca;
			const size_t step = (std::max<size_t>)(1, (fewer.size() * more.size()) / 20000000 + 1);
			float best = FLT_MAX;
			std::array<float, 3> ps{ 0, 0, 0 }, pl{ 0, 0, 0 };
			for (const auto& cs : fewer)
				for (size_t i = 0; i < more.size(); i += step)
				{
					const float d = dtVdist(cs.data(), more[i].data());
					if (d < best) { best = d; ps = cs; pl = more[i]; }
				}
			const auto& pa = (&fewer == &ca) ? ps : pl;
			const auto& pb = (&fewer == &ca) ? pl : ps;
			w.StartObject();
			w.Key("startRegionPolys"); w.Uint64(a.size());
			w.Key("pointRegionPolys"); w.Uint64(b.size());
			w.Key("gapDistance"); w.Double(Round1(best));
			w.Key("approximate"); w.Bool(step > 1);
			w.Key("startSide"); WriteLoc(w, pa.data());
			w.Key("pointSide"); WriteLoc(w, pb.data());
			w.EndObject();
		}
		else
		{
			w.Null();
		}
		w.EndObject();
		Emit(buffer);
		return 0;
	}

	int ConfigCommand(const Options& options)
	{
		fs::path iniPath;
		if (options.count("--ini"))
		{
			iniPath = PathOption(options, "--ini");
		}
		else
		{
			char exe[MAX_PATH] = { 0 };
			GetModuleFileNameA(nullptr, exe, MAX_PATH);
			iniPath = fs::path(exe).parent_path() / "MeshGenerator.ini";
		}
		std::error_code error;
		const bool exists = fs::is_regular_file(iniPath, error);
		char eqPath[MAX_PATH] = { 0 }, outPath[MAX_PATH] = { 0 }, extents[16] = { 0 };
		if (exists)
		{
			GetPrivateProfileStringA("General", "EverQuest Path", "", eqPath, MAX_PATH, iniPath.string().c_str());
			GetPrivateProfileStringA("General", "Output Path", "", outPath, MAX_PATH, iniPath.string().c_str());
			GetPrivateProfileStringA("General", "ZoneMaxExtents", "", extents, sizeof(extents), iniPath.string().c_str());
		}
		const fs::path meshFolder = std::string(outPath).empty() ? fs::path() : fs::path(outPath) / "MQ2Nav";

		rapidjson::StringBuffer buffer;
		Json w(buffer);
		w.StartObject();
		w.Key("ok"); w.Bool(true);
		w.Key("command"); w.String("config");
		w.Key("ini"); w.String(iniPath.string().c_str());
		w.Key("iniExists"); w.Bool(exists);
		w.Key("eqPath"); w.String(eqPath);
		w.Key("eqPathExists"); w.Bool(eqPath[0] && fs::is_directory(eqPath, error));
		w.Key("outputPath"); w.String(outPath);
		w.Key("outputPathExists"); w.Bool(outPath[0] && fs::is_directory(outPath, error));
		w.Key("meshFolder"); w.String(meshFolder.string().c_str());
		w.Key("zoneMaxExtents"); w.String(extents);
		w.Key("guiWillAskForFolders"); w.Bool(!eqPath[0] || !outPath[0]);
		w.EndObject();
		Emit(buffer);
		return 0;
	}
}

bool IsHeadlessCommand(const std::string& command)
{
	static const std::set<std::string> commands = {
		"build", "validate", "export", "diff", "path", "reach", "inspect", "probe", "config", "--help", "-h",
	};
	return commands.count(command) > 0;
}

int RunHeadlessCli(int argc, char* argv[])
{
	auto logger = spdlog::stderr_color_mt("MeshGeneratorCLI");
	spdlog::set_default_logger(logger);
	spdlog::set_pattern("%^%l:%$ %v");
	spdlog::set_level(spdlog::level::info);

	try
	{
		if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")
		{
			PrintUsage();
			return 0;
		}

		const std::string command = argv[1];
		if (command == "build")
		{
			std::set<std::string> values = { "--zone", "--eq-path", "--stage-root", "--mesh-root",
				"--preset", "--settings-from", "--carry-from", "--fixes", "--connections", "--volumes",
				"--base", "--tiles", "--box", "--auto-drops", "--max-drop", "--drop-spacing", "--threads", "--mesh-version" };
			for (const Setting& setting : kSettings)
				values.insert(setting.name);
			return BuildMesh(ParseOptions(argc, argv, values, { "--no-max-extents", "--walkways", "--allow-live" }));
		}
		if (command == "validate")
			return ValidateMesh(ParseOptions(argc, argv, { "--zone", "--mesh" }, {}));
		if (command == "export")
			return ExportFixes(ParseOptions(argc, argv, { "--zone", "--mesh", "--out" }, {}));
		if (command == "diff")
			return DiffMeshes(ParseOptions(argc, argv, { "--zone", "--old", "--new", "--tolerance" }, { "--strict" }));
		if (command == "path")
			return PathCommand(ParseOptions(argc, argv, { "--zone", "--mesh", "--from", "--to", "--extents" }, {}));
		if (command == "reach")
			return ReachCommand(ParseOptions(argc, argv, { "--zone", "--mesh", "--from", "--min-polys", "--limit", "--extents" }, {}));
		if (command == "inspect")
			return InspectCommand(ParseOptions(argc, argv, { "--zone", "--mesh", "--at", "--radius", "--extents" }, {}));
		if (command == "probe")
			return ProbeMesh(ParseOptions(argc, argv, { "--zone", "--mesh", "--points", "--routes", "--extents" }, {}));
		if (command == "config")
			return ConfigCommand(ParseOptions(argc, argv, { "--ini" }, {}));
		throw CliError(2, "unknown_command", "Unknown headless command: " + command);
	}
	catch (const CliError& error)
	{
		if (error.exitCode != 0)
			PrintError(error.code, error.what());
		return error.exitCode;
	}
	catch (const std::exception& error)
	{
		PrintError("internal_error", error.what());
		return 70;
	}
}
