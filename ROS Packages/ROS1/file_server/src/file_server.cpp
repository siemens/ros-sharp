/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

- Add path traversal and extension checks to prevent unauthorized file access.
- Add parameters to enable/disable saving and overwriting files (disabled by default).
- Remove auto package generation for security.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

- Add parameters to enable/disable file:// URLs for get_file, and to restrict them to a 
specific root directory (disabled by default).
- Add mutex and lock guards to make the get_file_callback thread-safe.
- Add optional 'localhost' authority stripping for file:// URLs.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <file_server/GetBinaryFile.h>
#include <file_server/SaveBinaryFile.h>

#include <fstream>
#include <filesystem>
#include <unordered_set>
#include <algorithm>
#include <optional>

const std::unordered_set<std::string>& allowed_extensions()
{
    static const std::unordered_set<std::string> exts =
    {
        ".urdf", ".xacro", ".stl", ".dae", ".obj", ".mtl",
        ".png",  ".jpg",   ".jpeg",".bmp", ".tga", ".yaml",
        ".tif",  ".tiff",  ".gif", ".material"
    };
    return exts;
}

bool is_extension_allowed(const std::string& path)
{
    auto ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return allowed_extensions().count(ext) > 0;
}

bool has_traversal(const std::string& path)
{
    for (const auto& part : std::filesystem::path(path))
        if (part == ".." || part == ".")
            return true;
    return false;
}

bool is_path_safe(const std::string& base_dir, const std::string& full_path)
{
    try
    {
        auto base   = std::filesystem::canonical(base_dir);
        auto target = std::filesystem::weakly_canonical(full_path);
        auto [end, _] = std::mismatch(base.begin(), base.end(), target.begin());
        return end == base.end();
    } catch (...)
    {
        return false;
    }
}

// `filepath` is whatever should be traversal/extension-checked (a package-relative sub-path
// or the full path itself for file://)
// `base_dir` is whatever full_path must stay inside.
// TODO: need better exception handling
bool validate_path(const std::string& filepath, const std::string& base_dir, const std::string& full_path)
{
    if (has_traversal(filepath))
    {
        ROS_WARN("Path traversal attempt blocked: %s", full_path.c_str());
        return false;
    }
    if (!is_extension_allowed(filepath))
    {
        ROS_WARN("Extension not allowed: %s", full_path.c_str());
        return false;
    }
    if (!is_path_safe(base_dir, full_path))
    {
        ROS_WARN("Path escapes allowed directory (%s): %s", base_dir.c_str(), full_path.c_str());
        return false;
    }
    return true;
}

// Single resolver used by both callbacks. `allow_file_scheme` is the only thing that
// differs between call sites: get_file passes the current allow_file_url parameter value,
// save_file always passes false, since writes to file:// targets are never supported.
// Returns the validated absolute path, or std::nullopt (with a warning already logged).
// TODO: need better exception handling
std::optional<std::string> resolve_address(const std::string& name, bool allow_file_scheme)
{
    if (name.compare(0, 10, "package://") == 0)
    {
        std::string address = name.substr(10);
        std::string package  = address.substr(0, address.find("/"));
        std::string filepath = address.substr(package.length());

        std::string base_dir = ros::package::getPath(package);
        if (base_dir.empty())
        {
            ROS_WARN("Package not found: %s", package.c_str());
            return std::nullopt;
        }

        std::string full_path = base_dir + filepath;
        if (!validate_path(filepath, base_dir, full_path)) return std::nullopt;
        return full_path;
    }

    if (name.compare(0, 7, "file://") == 0)
    {
        if (!allow_file_scheme)
        {
            ROS_WARN("\"file://\" address rejected here: %s. Enable \"allow_file_url\" parameter to allow.", name.c_str());
            return std::nullopt;
        }

        std::string path = name.substr(7);
        // Strip optional "localhost" authority: file://localhost/path -> /path
        if (path.compare(0, 9, "localhost") == 0)
            path = path.substr(9);

        std::string root;
        ros::param::param<std::string>("~allow_file_url_root", root, "/opt/ros/");

        if (!validate_path(path, root, path)) return std::nullopt;
        return path;
    }

    ROS_WARN("Only \"package://\" or \"file://\" addresses allowed: %s", name.c_str());
    return std::nullopt;
}

// callbacks

bool get_file(
    file_server::GetBinaryFile::Request  &req,
    file_server::GetBinaryFile::Response &res)
{
    bool allow_file_url = false;
    ros::param::get("~allow_file_url", allow_file_url);

    auto full_path = resolve_address(req.name, allow_file_url);
    if (!full_path) return true;

    std::ifstream file(*full_path, std::ios::binary);
    if (!file.is_open())
    {
        ROS_WARN("File not found: %s", full_path->c_str());
        return true;
    }

    res.value.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    ROS_INFO("get_file: %s", req.name.c_str());
    return true;
}

bool save_file(
    file_server::SaveBinaryFile::Request  &req,
    file_server::SaveBinaryFile::Response &res)
{
    bool allow_save = false;
    ros::param::get("~allow_save", allow_save);
    if (!allow_save)
    {
        ROS_WARN("Save service is disabled. Enable with parameter \"allow_save\".");
        return true;
    }

    // allow_file_scheme = false: writes never go through file://, no matter what.
    auto full_path = resolve_address(req.name, false);
    if (!full_path) return true;

    bool allow_overwrite = false;
    ros::param::get("~allow_overwrite", allow_overwrite);
    if (!allow_overwrite && std::filesystem::exists(*full_path))
    {
        ROS_WARN("Overwrite blocked: %s. Enable \"allow_overwrite\" parameter to allow.", full_path->c_str());
        return true;
    }

    std::filesystem::create_directories(std::filesystem::path(*full_path).parent_path());

    std::ofstream file(*full_path, std::ios::binary);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open for write: %s", full_path->c_str());
        return true;
    }

    file.write(reinterpret_cast<const char*>(req.value.data()), req.value.size());
    res.name = req.name;
    ROS_INFO("save_file: %s", req.name.c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "file_server");
    ros::NodeHandle n("~");

    bool allow_save      = false;
    bool allow_overwrite = false;
    bool allow_file_url  = false;
    std::string allow_file_url_root = "/opt/ros/";

    n.param("allow_save",          allow_save,          false);
    n.param("allow_overwrite",     allow_overwrite,     false);
    n.param("allow_file_url",      allow_file_url,      false);
    n.param("allow_file_url_root", allow_file_url_root, std::string("/opt/ros/"));

    if (allow_save)
        ROS_INFO("Save enabled. Overwrite: %s", allow_overwrite ? "yes" : "no");
    else
        ROS_INFO("Save disabled.");

    if (allow_file_url)
        ROS_WARN("\"file://\" URLs enabled for get_file, restricted to root: %s. "
                  "Only enable this if you trust the contents of that directory.",
                  allow_file_url_root.c_str());
    else
        ROS_INFO("Read access to file:// URLs is disabled.");

    ros::ServiceServer get_service  = n.advertiseService("/file_server/get_file",  get_file);
    ros::ServiceServer save_service = n.advertiseService("/file_server/save_file", save_file);

    ROS_INFO("ROS1 File Server initialized.");
    ros::spin();
    return 0;
}