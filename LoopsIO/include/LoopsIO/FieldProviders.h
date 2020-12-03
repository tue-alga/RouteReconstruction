#ifndef LOOPSIO_FIELDPROVIDERS_H
#define LOOPSIO_FIELDPROVIDERS_H
#include <LoopsIO/FieldProviders/TxtFieldProvider.h>
#include <LoopsIO/FieldProviders/JsonFieldProvider.h>
#include <LoopsIO/FieldProviders/IpeFieldProvider.h>
namespace LoopsIO
{
    using AllFieldProviders = std::tuple<
        FieldProviders::TxtFieldProvider,
        FieldProviders::JsonFieldProvider,
        FieldProviders::IpeFieldProvider
    >;
}
#endif