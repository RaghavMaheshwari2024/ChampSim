/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BLOCK_H
#define BLOCK_H

#include <cstdint>
#include <limits>

#include "access_type.h"
#include "champsim.h"

namespace champsim
{
struct cache_block {
  bool valid = false;
  bool prefetch = false;
  bool dirty = false;

  champsim::address address{};
  champsim::address v_address{};
  champsim::address data{};

  uint32_t pf_metadata = 0;

  uint64_t insertion_cycle = std::numeric_limits<uint64_t>::max();
  uint64_t last_access_cycle = std::numeric_limits<uint64_t>::max();
  uint64_t previous_set_access_count = std::numeric_limits<uint64_t>::max();
  uint64_t preuse_distance = std::numeric_limits<uint64_t>::max();
  uint64_t hits_since_insertion = 0;
  uint64_t dirty_since_cycle = std::numeric_limits<uint64_t>::max();
  access_type last_access_type = access_type::NUM_TYPES;
};
} // namespace champsim

#endif
