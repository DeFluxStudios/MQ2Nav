//
// HeadlessCli.cpp
//

#include "meshgen/HeadlessCli.h"

#include "common/NavMesh.h"
#include "common/NavMeshData.h"
#include "meshgen/InputGeom.h"
#include "meshgen/MapGeometryLoader.h"
#include "meshgen/NavMeshTool.h"

#include <array>
#include <cfloat>
#include <algorithm>
#include <cmath>
#include <vector>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>
#include <fmt/format.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <rapidjson/document.h>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <Windows.h>

namespace fs = std::filesystem;

namespace
{
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

	std::string JsonEscape(const std::string& value)
	{
		std::ostringstream escaped;
		for (unsigned char character : value)
		{
			switch (character)
			{
			case '"': escaped << "\\\""; break;
			case '\\': escaped << "\\\\"; break;
			case '\b': escaped << "\\b"; break;
			case '\f': escaped << "\\f"; break;
			case '\n': escaped << "\\n"; break;
			case '\r': escaped << "\\r"; break;
			case '\t': escaped << "\\t"; break;
			default:
				if (character < 0x20)
				{
					escaped << fmt::format("\\u{:04x}", static_cast<unsigned int>(character));
				}
				else
				{
					escaped << static_cast<char>(character);
				}
			}
		}
		return escaped.str();
	}

	void PrintError(const std::string& code, const std::string& message)
	{
		fmt::print("{{\"ok\":false,\"error\":\"{}\",\"message\":\"{}\"}}\n",
			JsonEscape(code), JsonEscape(message));
	}

	void PrintUsage()
	{
		fmt::print("{}",
			"MeshGenerator headless commands:\n"
			"  MeshGenerator.exe build --zone SHORTNAME --eq-path PATH --stage-root PATH [--mesh-root PATH] [--no-max-extents]\n"
			"      [--tile-size N] [--cell-size N] [--cell-height N] [--agent-height N] [--agent-radius N]\n"
			"      [--max-climb N] [--max-slope N] [--region-min-size N] [--region-merge-size N]\n"
			"      [--detail-sample-dist N] [--detail-sample-max-error N]   (defaults: MQ2Nav's own)\n"
			"      [--connections FILE]   JSON array of {\"start\":[x,y,z],\"end\":[x,y,z]} in mesh axes, y up\n"
			"      [--volumes FILE]       JSON array of {\"verts\":[[x,y,z],...],\"hmin\":N,\"hmax\":N,\"area\":N}\n"
			"  MeshGenerator.exe validate --zone SHORTNAME --mesh PATH\n"
			"  MeshGenerator.exe probe --zone SHORTNAME --mesh PATH --points \"Y,X,Z;Y,X,Z\"   /loc order, never writes\n"
			"\n"
			"Build writes <stage-root>\\MQ2Nav\\<zone>.navmesh and never overwrites an existing file.\n"
			"Progress and diagnostics are written to stderr; each command writes one JSON result to stdout.\n");
	}

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

	int CountTiles(const std::shared_ptr<dtNavMesh>& navMesh)
	{
		if (!navMesh)
			return 0;

		int count = 0;
		for (int index = 0; index < navMesh->getMaxTiles(); ++index)
		{
			const dtMeshTile* tile = const_cast<const dtNavMesh*>(navMesh.get())->getTile(index);
			if (tile && tile->header)
				++count;
		}
		return count;
	}

	int ValidateMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path meshPath = fs::absolute(RequiredOption(options, "--mesh")).lexically_normal();
		std::error_code error;
		if (!fs::is_regular_file(meshPath, error))
			throw CliError(3, "missing_mesh", "Mesh file not found: " + meshPath.string());

		NavMesh navMesh("", zone);
		const NavMesh::LoadResult result = navMesh.LoadNavMeshFile(meshPath.string());
		if (result != NavMesh::LoadResult::Success)
			throw CliError(5, "invalid_mesh", fmt::format("Mesh validation failed (load result {}).", static_cast<int>(result)));

		const int tileCount = CountTiles(navMesh.GetNavMesh());
		if (tileCount == 0)
			throw CliError(5, "empty_mesh", "Mesh loaded successfully but contains no tiles.");

		if (!navMesh.GetNavMeshQuery())
			throw CliError(5, "invalid_mesh", "Mesh loaded but its navigation query could not be initialized.");

		fmt::print("{{\"ok\":true,\"command\":\"validate\",\"zone\":\"{}\",\"mesh\":\"{}\",\"tiles\":{},\"headerVersion\":{}}}\n",
			JsonEscape(zone), JsonEscape(meshPath.string()), tileCount, navMesh.GetHeaderVersion());
		return 0;
	}

	int BuildMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path eqPath = fs::absolute(RequiredOption(options, "--eq-path")).lexically_normal();
		const fs::path stageRoot = fs::absolute(RequiredOption(options, "--stage-root")).lexically_normal();
		const fs::path meshRoot = fs::absolute(options.count("--mesh-root")
			? options.at("--mesh-root") : eqPath.string()).lexically_normal();

		std::error_code error;
		if (!fs::is_directory(eqPath, error))
			throw CliError(3, "missing_eq_path", "EverQuest geometry directory not found: " + eqPath.string());
		if (!fs::is_directory(meshRoot, error))
			throw CliError(3, "missing_mesh_root", "Mesh metadata root not found: " + meshRoot.string());
		if (stageRoot.string().size() >= MAX_PATH - 16)
			throw CliError(2, "path_too_long", "Stage root is too long for this MQ2Nav build.");

		const fs::path outputDirectory = stageRoot / "MQ2Nav";
		const fs::path outputPath = outputDirectory / (zone + ".navmesh");
		if (fs::exists(outputPath, error))
			throw CliError(4, "output_exists", "Refusing to overwrite staged mesh: " + outputPath.string());
		if (error)
			throw CliError(4, "output_error", "Could not inspect staged output path: " + error.message());

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

		// Optional settings. Anything not given keeps the MQ2Nav default.
		{
			NavMeshConfig& config = navMeshTool.GetConfig();
			struct Setting { const char* name; float* field; float low; float high; bool whole; };
			// Limits follow the MeshGenerator settings sliders, so the CLI can't build what the GUI can't.
			const Setting settings[] = {
				{ "--tile-size", &config.tileSize, 16.0f, 1024.0f, true },
				{ "--cell-size", &config.cellSize, 0.1f, 1.0f, false },
				{ "--cell-height", &config.cellHeight, 0.1f, 1.0f, false },
				{ "--agent-height", &config.agentHeight, 0.1f, 15.0f, false },
				{ "--agent-radius", &config.agentRadius, 0.1f, 15.0f, false },
				{ "--max-climb", &config.agentMaxClimb, 0.1f, 15.0f, false },
				{ "--max-slope", &config.agentMaxSlope, 0.0f, 90.0f, false },
				{ "--region-min-size", &config.regionMinSize, 0.0f, 150.0f, false },
				{ "--region-merge-size", &config.regionMergeSize, 0.0f, 150.0f, false },
				{ "--detail-sample-dist", &config.detailSampleDist, 0.0f, 0.9f, false },
				{ "--detail-sample-max-error", &config.detailSampleMaxError, 0.0f, 100.0f, false },
			};
			for (const Setting& setting : settings)
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
				*setting.field = value;
			}
			navMesh->GetNavMeshConfig() = config;
			spdlog::info("Settings: tile {} cell {} cellHeight {} height {} radius {} climb {} slope {}",
				config.tileSize, config.cellSize, config.cellHeight, config.agentHeight,
				config.agentRadius, config.agentMaxClimb, config.agentMaxSlope);
		}

		// Optional off-mesh links, baked into the tiles by the build. File: a JSON array of
		// {"start":[x,y,z],"end":[x,y,z],"bidirectional":true,"name":"..."} in mesh axes (y is up).
		int connectionCount = 0;
		if (const auto found = options.find("--connections"); found != options.end())
		{
			std::ifstream file(found->second, std::ios::binary);
			if (!file)
				throw CliError(3, "missing_connections", "Connections file not found: " + found->second);
			const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
			rapidjson::Document document;
			document.Parse(text.c_str());
			if (document.HasParseError() || !document.IsArray())
				throw CliError(2, "bad_connections", "Connections file must be a JSON array.");
			auto readPoint = [](const rapidjson::Value& item, const char* key, glm::vec3& out) {
				if (!item.HasMember(key) || !item[key].IsArray() || item[key].Size() != 3)
					return false;
				for (rapidjson::SizeType i = 0; i < 3; ++i)
				{
					if (!item[key][i].IsNumber())
						return false;
					out[i] = item[key][i].GetFloat();
				}
				return true;
			};
			for (const auto& item : document.GetArray())
			{
				auto connection = std::make_unique<OffMeshConnection>();
				if (!item.IsObject() || !readPoint(item, "start", connection->start) || !readPoint(item, "end", connection->end))
					throw CliError(2, "bad_connections", "Each connection needs start and end as [x,y,z].");
				if (item.HasMember("bidirectional") && item["bidirectional"].IsBool())
					connection->bidirectional = item["bidirectional"].GetBool();
				if (item.HasMember("name") && item["name"].IsString())
					connection->name = item["name"].GetString();
				navMesh->AddConnection(std::move(connection));
				++connectionCount;
			}
			spdlog::info("Added {} off-mesh connections.", connectionCount);
		}

		// Optional area volumes (water, avoid, and so on). File: a JSON array of
		// {"verts":[[x,y,z],...],"hmin":N,"hmax":N,"area":N,"name":"..."} in mesh axes (y is up).
		if (const auto found = options.find("--volumes"); found != options.end())
		{
			std::ifstream file(found->second, std::ios::binary);
			if (!file)
				throw CliError(3, "missing_volumes", "Volumes file not found: " + found->second);
			const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
			rapidjson::Document document;
			document.Parse(text.c_str());
			if (document.HasParseError() || !document.IsArray())
				throw CliError(2, "bad_volumes", "Volumes file must be a JSON array.");
			int volumeCount = 0;
			for (const auto& item : document.GetArray())
			{
				if (!item.IsObject() || !item.HasMember("verts") || !item["verts"].IsArray() || item["verts"].Size() < 3
					|| !item.HasMember("hmin") || !item["hmin"].IsNumber() || !item.HasMember("hmax") || !item["hmax"].IsNumber()
					|| !item.HasMember("area") || !item["area"].IsUint())
					throw CliError(2, "bad_volumes", "Each volume needs verts (3 or more), hmin, hmax and area.");
				std::vector<glm::vec3> verts;
				for (const auto& v : item["verts"].GetArray())
				{
					if (!v.IsArray() || v.Size() != 3 || !v[0].IsNumber() || !v[1].IsNumber() || !v[2].IsNumber())
						throw CliError(2, "bad_volumes", "Volume vertices must be [x,y,z].");
					verts.emplace_back(v[0].GetFloat(), v[1].GetFloat(), v[2].GetFloat());
				}
				// Recast keeps areas in 6 bits; 0 is "not walkable", which a volume can't set here.
				const unsigned int area = item["area"].GetUint();
				if (area < 1 || area > 63)
					throw CliError(2, "bad_volumes", "Volume area must be from 1 to 63.");
				const float hmin = item["hmin"].GetFloat(), hmax = item["hmax"].GetFloat();
				if (!(hmin < hmax))
					throw CliError(2, "bad_volumes", "Volume hmin must be below hmax.");
				const std::string name = item.HasMember("name") && item["name"].IsString() ? item["name"].GetString() : "";
				navMesh->AddConvexVolume(verts, name, hmin, hmax, static_cast<uint8_t>(area));
				++volumeCount;
			}
			spdlog::info("Added {} area volumes.", volumeCount);
		}

		navMeshTool.UpdateTileSizes();
		spdlog::info("Building zone {} synchronously.", zone);
		if (!navMeshTool.handleBuild(false, false))
			throw CliError(4, "build_failed", "MQ2Nav failed to initialize the navmesh build.");

		const int tileCount = CountTiles(navMesh->GetNavMesh());
		if (tileCount == 0)
			throw CliError(4, "empty_build", "Build completed without producing any navmesh tiles.");

		const fs::path temporaryPath = outputPath.string() + ".building";
		if (fs::exists(temporaryPath, error))
			throw CliError(4, "temporary_exists", "Refusing to overwrite temporary build file: " + temporaryPath.string());

		if (!navMesh->SaveNavMeshFile(temporaryPath.string()))
		{
			fs::remove(temporaryPath, error);
			throw CliError(4, "save_failed", "Could not save the staged mesh.");
		}

		NavMesh validation("", zone);
		if (validation.LoadNavMeshFile(temporaryPath.string()) != NavMesh::LoadResult::Success
			|| CountTiles(validation.GetNavMesh()) != tileCount
			|| !validation.GetNavMeshQuery())
		{
			fs::remove(temporaryPath, error);
			throw CliError(5, "staged_validation_failed", "The staged mesh did not pass MQ2Nav load/query validation.");
		}

		fs::rename(temporaryPath, outputPath);
		fmt::print("{{\"ok\":true,\"command\":\"build\",\"zone\":\"{}\",\"mesh\":\"{}\",\"tiles\":{},\"validation\":\"passed\"}}\n",
			JsonEscape(zone), JsonEscape(outputPath.string()), tileCount);
		return 0;
	}
	// Reads a mesh and reports, using MQ2Nav's own search box, whether each point is on it and
	// reachable from the first point. Points are /loc order: "y,x,z;y,x,z". Never writes anything.
	int ProbeMesh(const Options& options)
	{
		const std::string zone = GetZone(options);
		const fs::path meshPath = fs::absolute(RequiredOption(options, "--mesh")).lexically_normal();
		NavMesh navMesh("", zone);
		if (navMesh.LoadNavMeshFile(meshPath.string()) != NavMesh::LoadResult::Success)
			throw CliError(5, "invalid_mesh", "Mesh could not be loaded.");
		auto query = navMesh.GetNavMeshQuery();
		if (!query)
			throw CliError(5, "invalid_mesh", "No navigation query.");

		// Read every point before printing anything, so a bad point gives one error and no partial output.
		std::vector<std::array<float, 3>> locs;
		{
			std::stringstream points(RequiredOption(options, "--points"));
			std::string item;
			while (std::getline(points, item, ';'))
			{
				float y = 0, x = 0, z = 0;
				if (sscanf_s(item.c_str(), "%f,%f,%f", &y, &x, &z) != 3
					|| !std::isfinite(y) || !std::isfinite(x) || !std::isfinite(z))
					throw CliError(2, "bad_points", "Points must be y,x,z;y,x,z");
				locs.push_back({ y, x, z });
			}
			if (locs.empty())
				throw CliError(2, "bad_points", "Points must be y,x,z;y,x,z");
		}

		// probe prints several lines: this header (ok and command), one line per point, and a gap line
		// when a route is broken.
		const auto& c = navMesh.GetNavMeshConfig();
		fmt::print("{{\"ok\":true,\"command\":\"probe\",\"mesh\":\"{}\",\"tiles\":{},\"cellSize\":{},\"cellHeight\":{},\"tileSize\":{},\"agentHeight\":{},\"agentRadius\":{},\"agentMaxClimb\":{},\"agentMaxSlope\":{}}}\n",
			JsonEscape(meshPath.string()), CountTiles(navMesh.GetNavMesh()), c.cellSize, c.cellHeight, c.tileSize,
			c.agentHeight, c.agentRadius, c.agentMaxClimb, c.agentMaxSlope);

		dtQueryFilter filter;
		filter.setIncludeFlags(+PolyFlags::All);
		filter.setExcludeFlags(+PolyFlags::Disabled);
		navMesh.FillFilterAreaCosts(filter);

		const float nearBox[3] = { 5.f, 10.f, 5.f };
		const float wide[3] = { 60.f, 60.f, 60.f };
		dtPolyRef startRef = 0;
		float startPos[3] = { 0, 0, 0 };
		bool first = true;
		bool gapReported = false;
		for (const auto& loc : locs)
		{
			const float y = loc[0], x = loc[1], z = loc[2];
			const float pos[3] = { x, z, y };
			dtPolyRef ref = 0, wideRef = 0;
			float hit[3] = { 0, 0, 0 }, wideHit[3] = { 0, 0, 0 };
			query->findNearestPoly(pos, nearBox, &filter, &ref, hit);
			query->findNearestPoly(pos, wide, &filter, &wideRef, wideHit);
			if (first) { startRef = ref; dtVcopy(startPos, hit); first = false; }

			// no_start: the first point is off the mesh, so no route can be tested.
			// search_limit: Detour ran out of nodes or path space, which is not proof of a break.
			std::string reach = !startRef ? "no_start" : "not_on_mesh";
			int polys = 0;
			std::string endForward = "null", endBack = "null";
			if (ref && startRef)
			{
				dtPolyRef path[4096];
				const dtStatus status = query->findPath(startRef, ref, startPos, hit, &filter, path, &polys, 4096);
				if (dtStatusFailed(status))
					reach = "failed";
				else if (polys > 0 && path[polys - 1] == ref)
					reach = "reachable";
				else if (dtStatusDetail(status, DT_OUT_OF_NODES) || dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
					reach = "search_limit";
				else
					reach = "partial";
				if (reach == "partial")
				{
					// Where each side gets closest to the other: the break lies between these two.
					float end[3] = { 0, 0, 0 };
					if (dtStatusSucceed(query->closestPointOnPoly(path[polys - 1], hit, end, nullptr)))
						endForward = fmt::format("[{:.1f},{:.1f},{:.1f}]", end[2], end[0], end[1]);
					int backPolys = 0;
					if (dtStatusSucceed(query->findPath(ref, startRef, hit, startPos, &filter, path, &backPolys, 4096)) && backPolys > 0
						&& dtStatusSucceed(query->closestPointOnPoly(path[backPolys - 1], startPos, end, nullptr)))
						endBack = fmt::format("[{:.1f},{:.1f},{:.1f}]", end[2], end[0], end[1]);
				}
			}
			fmt::print("{{\"y\":{},\"x\":{},\"z\":{},\"onMesh\":{},\"route\":\"{}\",\"polys\":{},\"nearestWithin60\":{},\"startSideEnd\":{},\"pointSideEnd\":{}}}\n",
				y, x, z, ref != 0, reach, polys,
				wideRef ? fmt::format("[{:.1f},{:.1f},{:.1f}]", wideHit[2], wideHit[0], wideHit[1]) : "null", endForward, endBack);

			if (reach == "partial" && !gapReported)
			{
				gapReported = true;
				const dtNavMesh* mesh = query->getAttachedNavMesh();
				// Same filter as findPath, so the two regions agree with the route search.
				const auto flood = [mesh, &filter](dtPolyRef from) {
					std::vector<std::array<float, 3>> centres;
					std::set<dtPolyRef> seen{ from };
					std::vector<dtPolyRef> open{ from };
					while (!open.empty())
					{
						const dtPolyRef cur = open.back();
						open.pop_back();
						const dtMeshTile* tile = nullptr;
						const dtPoly* poly = nullptr;
						mesh->getTileAndPolyByRefUnsafe(cur, &tile, &poly);
						std::array<float, 3> c{ 0, 0, 0 };
						for (int v = 0; v < poly->vertCount; ++v)
							for (int a = 0; a < 3; ++a)
								c[a] += tile->verts[poly->verts[v] * 3 + a] / poly->vertCount;
						centres.push_back(c);
						for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = tile->links[l].next)
						{
							const dtPolyRef next = tile->links[l].ref;
							const dtMeshTile* nextTile = nullptr;
							const dtPoly* nextPoly = nullptr;
							if (!next || seen.count(next)) continue;
							mesh->getTileAndPolyByRefUnsafe(next, &nextTile, &nextPoly);
							if (!filter.passFilter(next, nextTile, nextPoly)) continue;
							seen.insert(next);
							open.push_back(next);
						}
					}
					return centres;
				};
				const auto a = flood(startRef), b = flood(ref);
				// Every pair is compared up to about 20 million checks. Past that, the larger
				// side is sampled evenly and the result is marked approximate.
				const auto& fewer = a.size() <= b.size() ? a : b;
				const auto& more = a.size() <= b.size() ? b : a;
				const size_t budget = 20000000;
				const size_t step = (std::max<size_t>)(1, (fewer.size() * more.size()) / budget + 1);
				float best = FLT_MAX;
				std::array<float, 3> ps{ 0, 0, 0 }, pl{ 0, 0, 0 };
				for (const auto& cs : fewer)
					for (size_t i = 0; i < more.size(); i += step)
					{
						const float d = dtVdist(cs.data(), more[i].data());
						if (d < best) { best = d; ps = cs; pl = more[i]; }
					}
				const auto& pa = (&fewer == &a) ? ps : pl;
				const auto& pb = (&fewer == &a) ? pl : ps;
				fmt::print("{{\"startRegionPolys\":{},\"pointRegionPolys\":{},\"gapDistance\":{:.1f},\"approximate\":{},\"startSide\":[{:.1f},{:.1f},{:.1f}],\"pointSide\":[{:.1f},{:.1f},{:.1f}]}}\n",
					a.size(), b.size(), best, step > 1, pa[2], pa[0], pa[1], pb[2], pb[0], pb[1]);
			}
		}
		return 0;
	}
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
			const Options options = ParseOptions(argc, argv,
				{ "--zone", "--eq-path", "--stage-root", "--mesh-root",
					"--tile-size", "--cell-size", "--cell-height", "--agent-height", "--agent-radius",
					"--max-climb", "--max-slope", "--region-min-size", "--region-merge-size",
					"--detail-sample-dist", "--detail-sample-max-error", "--connections", "--volumes" },
				{ "--no-max-extents" });
			return BuildMesh(options);
		}
		if (command == "validate")
		{
			const Options options = ParseOptions(argc, argv, { "--zone", "--mesh" }, {});
			return ValidateMesh(options);
		}

		if (command == "probe")
		{
			const Options options = ParseOptions(argc, argv, { "--zone", "--mesh", "--points" }, {});
			return ProbeMesh(options);
		}
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
