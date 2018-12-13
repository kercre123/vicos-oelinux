#include "util.h"

#include <fstream>

namespace anki {

void dump_frame(const std::string& filename, const uint8_t *bytes, uint32_t num_bytes)
{
  std::ofstream ofs(filename, std::ios::binary);
  ofs.write(reinterpret_cast<const char*>(bytes), num_bytes);
  ofs.close();
}

} /* namespace anki */