#ifndef _mm_pdoran_util_h_
#define _mm_pdoran_util_h_

#include <string>

namespace anki
{

/**
 * @brief Dump binary data to a file
 */
void dump_frame(const std::string& filename, const uint8_t *bytes, uint32_t num_bytes);

} /* namespace anki */

#endif