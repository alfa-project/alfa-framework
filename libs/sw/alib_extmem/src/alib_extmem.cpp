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

#include "alib_extmem.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace std;

alib_extmem::alib_extmem() {
  if ((this->ext_mem_fd = open("/dev/alfa_ext_mem", O_RDWR | O_SYNC)) != -1) {
    this->ext_mem = (uint64_t*)mmap(NULL, EXTMEM_SIZE, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, ext_mem_fd, 0);
  } else {
    printf("Failed to open /dev/alfa_ext_mem\n");
    exit(1);
  }
}

alib_extmem::~alib_extmem() {
  if (ext_mem_fd != -1) close(this->ext_mem_fd);
}

void alib_extmem::read_memory(uint32_t address, size_t size,
                              void* buffer) {
  // Read memory from external memory
  memcpy(buffer, (void*)(this->ext_mem + address), size);
}