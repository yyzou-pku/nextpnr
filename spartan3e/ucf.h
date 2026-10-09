#ifndef SPARTAN3E_UCF_H
#define SPARTAN3E_UCF_H
#include <string>
#include "nextpnr_namespaces.h"
NEXTPNR_NAMESPACE_BEGIN
struct Context;
struct UcfSource
{
    std::string filename;
    int line;
    std::string value;
};
void read_ucf(Context *ctx, const std::string &filename);
void report_ucf_context(Context *ctx, const char *phase);
NEXTPNR_NAMESPACE_END
#endif
