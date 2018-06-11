/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#ifndef COSIMA_GAZEBO_SPRING_CONSTRAINT_MAPPING_H_
#define COSIMA_GAZEBO_SPRING_CONSTRAINT_MAPPING_H_

#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <mutex>

#include <gazebo/common/common.hh>

namespace cosima
{

class SpringConstraintMapping
{
  public:
    SpringConstraintMapping()
    {
    }

    static SpringConstraintMapping &Get()
    {
        static SpringConstraintMapping instance;
        return instance;
    }

    struct Constraint
    {
        std::string name;
        gazebo::math::Pose constraintPose;
        bool in_world_frame;
        std::mutex constraint_mutex;
    };

    typedef std::shared_ptr<Constraint> ConstraintPtr;
    // oder auch ein pointer zum vector...?
    typedef std::vector<ConstraintPtr> ConstraintPtrVec;

    void Add(std::string name, ConstraintPtr constraint)
    {
        if (std::find(this->mapping[name].begin(), this->mapping[name].end(), constraint) != this->mapping[name].end())
        {
            // Element in vector -> already in list!
        }
        else
        {
            std::lock_guard<std::mutex> lock(spring_constraint_mapping_mutex);
            this->mapping[name].push_back(constraint);
        }
    }

    bool LinkContained(std::string name)
    {
        std::lock_guard<std::mutex> lock(spring_constraint_mapping_mutex);
        if (this->mapping.find(name) == this->mapping.end())
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void Remove(std::string name, ConstraintPtr constraint)
    {
        std::lock_guard<std::mutex> lock(spring_constraint_mapping_mutex);
        this->mapping[name].erase(std::remove(this->mapping[name].begin(), this->mapping[name].end(), constraint), this->mapping[name].end());
    }

    ConstraintPtrVec GetConstraintVec(std::string name)
    {
        std::lock_guard<std::mutex> lock(spring_constraint_mapping_mutex);
        return this->mapping[name];
    }

    std::unordered_map<std::string, ConstraintPtrVec> mapping;

    std::mutex spring_constraint_mapping_mutex;
};
} // namespace cosima

#endif // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_CONTAINER_H_