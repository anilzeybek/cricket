#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <map>

int main(int argc, char **argv)
{
    using namespace pinocchio;

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_urdf> <path_to_srdf>" << std::endl;
        std::cerr << "Example: " << argv[0] << " robot.urdf robot.srdf" << std::endl;
        return -1;
    }

    const std::string urdf_filename = argv[1];
    const std::string srdf_filename = argv[2];

    try
    {
        // Load the URDF model
        std::cout << "Loading URDF model from: " << urdf_filename << std::endl;
        Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);

        // Load the geometries associated to model which are contained in the URDF file
        std::cout << "Loading collision geometries..." << std::endl;
        GeometryModel geom_model;
        pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model);

        // Add all possible collision pairs
        std::cout << "Adding all collision pairs..." << std::endl;
        geom_model.addAllCollisionPairs();
        std::cout << "Initial number of collision pairs: " << geom_model.collisionPairs.size() << std::endl;

        // Remove collision pairs listed in the SRDF file
        std::cout << "Loading SRDF and removing disabled collision pairs from: " << srdf_filename
                  << std::endl;
        pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);
        std::cout << "Number of allowed collision pairs after SRDF filtering: "
                  << geom_model.collisionPairs.size() << std::endl;

        // Create a map from link names to their frame indices for efficient lookup
        std::map<std::string, FrameIndex> link_name_to_index;
        for (FrameIndex i = 0; i < model.frames.size(); ++i)
        {
            if (model.frames[i].type == BODY)
            {
                link_name_to_index[model.frames[i].name] = i;
            }
        }

        // Collect all allowed collision pairs in terms of link indices
        std::set<std::pair<FrameIndex, FrameIndex>> allowed_link_pairs;
        std::map<std::string, std::set<std::string>> link_collision_map;

        std::cout << "\n=== ALLOWED COLLISION PAIRS ===" << std::endl;
        std::cout << "Geometry level pairs:" << std::endl;
        std::cout << "--------------------" << std::endl;

        for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
        {
            const CollisionPair &cp = geom_model.collisionPairs[k];

            // Get the geometry objects
            const GeometryObject &geom1 = geom_model.geometryObjects[cp.first];
            const GeometryObject &geom2 = geom_model.geometryObjects[cp.second];

            // Get the parent frame names (which correspond to links)
            const std::string &link1_name = model.frames[geom1.parentFrame].name;
            const std::string &link2_name = model.frames[geom2.parentFrame].name;

            // Get frame indices
            FrameIndex link1_idx = geom1.parentFrame;
            FrameIndex link2_idx = geom2.parentFrame;

            // Store the link pair (ensure consistent ordering)
            FrameIndex first_idx = std::min(link1_idx, link2_idx);
            FrameIndex second_idx = std::max(link1_idx, link2_idx);
            allowed_link_pairs.insert(std::make_pair(first_idx, second_idx));

            // Store for summary
            link_collision_map[link1_name].insert(link2_name);
            link_collision_map[link2_name].insert(link1_name);

            std::cout << "Geometry pair " << k << ": ";
            std::cout << "(" << cp.first << ", " << cp.second << ") ";
            std::cout << "-> Links: '" << link1_name << "' (idx:" << link1_idx << ") ";
            std::cout << "<-> '" << link2_name << "' (idx:" << link2_idx << ")" << std::endl;
        }

        // Print unique link pairs
        std::cout << "\nLink level pairs (unique):" << std::endl;
        std::cout << "-------------------------" << std::endl;

        int pair_count = 0;
        for (const auto &link_pair : allowed_link_pairs)
        {
            const std::string &link1_name = model.frames[link_pair.first].name;
            const std::string &link2_name = model.frames[link_pair.second].name;

            std::cout << "Pair " << pair_count++ << ": ";
            std::cout << "(" << link_pair.first << ", " << link_pair.second << ") ";
            std::cout << "-> '" << link1_name << "' <-> '" << link2_name << "'" << std::endl;
        }

        // Summary
        std::cout << "\n=== SUMMARY ===" << std::endl;
        std::cout << "Total number of allowed geometry collision pairs: " << geom_model.collisionPairs.size()
                  << std::endl;
        std::cout << "Total number of unique link collision pairs: " << allowed_link_pairs.size()
                  << std::endl;

        if (allowed_link_pairs.empty())
        {
            std::cout << "No collision pairs are allowed (all collisions disabled by SRDF)." << std::endl;
        }
        else
        {
            std::cout << "\nLinks allowed to collide:" << std::endl;
            std::cout << "------------------------" << std::endl;

            for (const auto &link_entry : link_collision_map)
            {
                const std::string &link_name = link_entry.first;
                const std::set<std::string> &collision_partners = link_entry.second;

                auto it = link_name_to_index.find(link_name);
                FrameIndex link_index = (it != link_name_to_index.end()) ? it->second : 0;

                std::cout << "Link '" << link_name << "' (index: " << link_index << ") can collide with: ";
                bool first = true;
                for (const auto &partner : collision_partners)
                {
                    if (!first)
                    {
                        std::cout << ", ";
                    }
                    std::cout << "'" << partner << "'";
                    first = false;
                }
                std::cout << std::endl;
            }

            std::cout << "\nLink index pairs (for programmatic use):" << std::endl;
            std::cout << "---------------------------------------" << std::endl;
            for (const auto &pair : allowed_link_pairs)
            {
                std::cout << "(" << pair.first << ", " << pair.second << ")" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
