/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ALIB_EXTMEM_H
#define ALIB_EXTMEM_H

#include "alfa_node.hpp"

#define EXTMEM_SIZE 0xA00000

using namespace std;

class alib_extmem {
 public:
  alib_extmem();
  ~alib_extmem();

  void read_memory(uint32_t offset, std::size_t size, void* buffer);

 private:
  int ext_mem_fd;
  uint64_t* ext_mem;
};

#endif  // ALIB_EXTMEM_H