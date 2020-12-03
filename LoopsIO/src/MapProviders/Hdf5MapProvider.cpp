#include <LoopsIO/MapProviders/Hdf5MapProvider.h>
#include <H5Cpp.h>

void LoopsIO::MapProviders::Hdf5MapProvider::write(const std::string& outputPath,
    const LoopsLib::DS::EmbeddedGraph& decompObj)
{
    H5::H5File file(outputPath.c_str(), H5F_ACC_TRUNC);

    auto group = file.createGroup("/Graph");
    // Add spatial reference string
    {
        auto crsAttribute = group.createAttribute("crs", H5::StrType(), H5S_SCALAR);
        char* data;
        decompObj.spatialRef().exportToWkt(&data);
        crsAttribute.write(H5::StrType(), data);
        CPLFree(data);
    }
    

    // Write locations
    {
        hsize_t      dims[2] = { decompObj.locations().size(), 2 };  // dataset dimensions at creation
        hsize_t      maxdims[2] = { H5S_UNLIMITED, 2 };
        H5::DataSpace mspace1(2, dims, maxdims);

        auto dataset = group.createDataSet("locations", H5::PredType::NATIVE_LDOUBLE, mspace1);

        auto data = new LoopsLib::KernelDef::NT[decompObj.locations().size() * 2];
        for (auto i = 0; i < decompObj.locations().size(); ++i)
        {
            data[2 * i] = decompObj.locations()[i].m_x;
            data[2 * i + 1] = decompObj.locations()[i].m_y;
        }
        dataset.write(data, H5::PredType::NATIVE_LDOUBLE);
        delete[] data;
    }

    //// Write edge connectivity
    {
        hsize_t      dims[2] = { decompObj.number_of_edges(), 2 };  // dataset dimensions at creation
        hsize_t      maxdims[2] = { H5S_UNLIMITED, 2 };
        H5::DataSpace mspace1(2, dims, maxdims);
        auto edgesDS = group.createDataSet("edges", H5::PredType::NATIVE_INT64, mspace1);
        auto data = new long long[decompObj.number_of_edges() * 2];
        for (auto i = 0; i < decompObj.number_of_edges(); ++i)
        {
            data[2 * i] = decompObj.edge(i)->m_source->id();
            data[2 * i + 1] = decompObj.edge(i)->m_sink->id();
        }
        edgesDS.write(data, H5::PredType::NATIVE_INT64);
        delete[] data;
    }

    //// 
}

void LoopsIO::MapProviders::Hdf5MapProvider::read(const std::string& inputPath, LoopsLib::DS::EmbeddedGraph& decompObj)
{
    H5::H5File file(inputPath.c_str(), H5F_ACC_RDONLY);

    auto ds = file.openDataSet("/Graph/locations");
    auto space = ds.getSpace();
    hsize_t dims[2];
    auto dimCount = space.getSimpleExtentDims(dims);
    assert(dimCount == 2);

    //ds.read(, H5::PredType::NATIVE_LDOUBLE);
    
    auto group = file.createGroup("/Graph");
    // Add spatial reference string
    {
        auto crsAttribute = group.createAttribute("crs", H5::StrType(), H5S_SCALAR);
        char* data;
        decompObj.spatialRef().exportToWkt(&data);
        crsAttribute.write(H5::StrType(), data);
        CPLFree(data);
    }


    // Write locations
    {
        hsize_t      dims[2] = { decompObj.locations().size(), 2 };  // dataset dimensions at creation
        hsize_t      maxdims[2] = { H5S_UNLIMITED, 2 };
        H5::DataSpace mspace1(2, dims, maxdims);

        auto dataset = group.createDataSet("locations", H5::PredType::NATIVE_LDOUBLE, mspace1);

        auto data = new LoopsLib::KernelDef::NT[decompObj.locations().size() * 2];
        for (auto i = 0; i < decompObj.locations().size(); ++i)
        {
            data[2 * i] = decompObj.locations()[i].m_x;
            data[2 * i + 1] = decompObj.locations()[i].m_y;
        }
        dataset.write(data, H5::PredType::NATIVE_LDOUBLE);
        delete[] data;
    }

    //// Write edge connectivity
    {
        hsize_t      dims[2] = { decompObj.number_of_edges(), 2 };  // dataset dimensions at creation
        hsize_t      maxdims[2] = { H5S_UNLIMITED, 2 };
        H5::DataSpace mspace1(2, dims, maxdims);
        auto edgesDS = group.createDataSet("edges", H5::PredType::NATIVE_INT64, mspace1);
        auto data = new long long[decompObj.number_of_edges() * 2];
        for (auto i = 0; i < decompObj.number_of_edges(); ++i)
        {
            data[2 * i] = decompObj.edge(i)->m_source->id();
            data[2 * i + 1] = decompObj.edge(i)->m_sink->id();
        }
        edgesDS.write(data, H5::PredType::NATIVE_INT64);
        delete[] data;
    }

    //// 
}
