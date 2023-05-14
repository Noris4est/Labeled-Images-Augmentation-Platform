#include <iostream>
#include <opencv2/opencv.hpp>
#include "warp_mesh_applicator.hpp"
#include "darknet_dataset.hpp"
#include "random_processing.hpp"
#include "distortion_mesh_generator_zoo.hpp"
#include <iomanip>
#include "utils.hpp"

    /*
        1. Случайное смещение вершин сетки random_move
        2. SQRT Left2Right
        3. SQRT TopLeft2BottomRight
        4. Optical Distortion 
        5. Transverse waves (sin, random angle)
        6. Longitudinal (gamma,sin | plane,cylindrical | random angle)
        
        Собираются рандомные пайплайны длиной pipeline_max_size,
        с рандомными характеристиками компонентов пайплайна
    */
enum MeshDistortionMode
{
    RandomMoveMeshNodes,
    // SQRT_left2right,
    // SQRT_topLeft2bottomRight,
    OpticalDistortion,
    TransverseWavePlane,
    LongitudinalWavePlane,
    LongitudinalWaveCylindrical,
    ALWAYS_LAST_ELEMENT_NOT_UP
};

bool getRandomWarpMesh(const cv::Mat primeMesh, cv::Mat &warpMesh, cv::Size workFrameSize, cv::Size meshGridSize, int number_attempts)
{
    int total_warp_mesh_modes = static_cast<int>(MeshDistortionMode::ALWAYS_LAST_ELEMENT_NOT_UP - 1);
    int current_warp_mode = rnd(0, total_warp_mesh_modes - 1);

    int try_counter = 0;
    bool success_create_valid_warp_mesh = false;
    while (try_counter < number_attempts)
    {
        switch (current_warp_mode)
        {
        case MeshDistortionMode::RandomMoveMeshNodes:
            {
                mesh_generator::random_move::createRandomWarpMesh(
                    primeMesh, warpMesh, workFrameSize, meshGridSize, true, 0, {0.1f, 0.3f});
            }
            break;
        // case MeshDistortionMode::SQRT_left2right:
        //     {
        //         mesh_generator::various_others::creatSqrtWarpMeshFromLeft2Right(
        //             primeMesh, warpMesh);
        //     }
        //     break;
        // case MeshDistortionMode::SQRT_topLeft2bottomRight:
        //     {
        //         mesh_generator::various_others::creatSqrtWarpMeshFromTopLeft2BottomRight(
        //             primeMesh, warpMesh);
        //     }
        //     break;
        case MeshDistortionMode::OpticalDistortion:
            {
                mesh_generator::optical_distortion::OpticalDistortionSettings dist_settings;
                dist_settings.k1 = rnd(-0.2f, -0.5f);
                mesh_generator::optical_distortion::createDistortionWarpMesh(
                    primeMesh, warpMesh, workFrameSize, meshGridSize, dist_settings);
            }
            break;
        case MeshDistortionMode::TransverseWavePlane:
            {
                int dice_value = rnd(0,2);
                float scale_factor = rnd(0.0f, 0.3f);
                switch (dice_value)
                {
                    case 0:
                        {
                            mesh_generator::transverse::createTransverseSinWarpMesh_propX_changeY(
                                primeMesh, warpMesh, workFrameSize, meshGridSize, scale_factor);
                        }
                        break;
                    case 1:
                        {
                            mesh_generator::transverse::createTransverseSinWarpMesh_propY_changeX(
                                primeMesh, warpMesh, workFrameSize, meshGridSize, scale_factor);
                        }
                        break;
                    case 2:
                        {
                            mesh_generator::transverse::createTransverseSinWarpMesh_propXY_changeXY(
                                primeMesh, warpMesh, workFrameSize, meshGridSize, scale_factor);
                        }
                        break;
                    default:
                        {
                            throw std::runtime_error("Error: not supported dice_value (transverse wave)");
                        }
                }
            }
            break;
        case MeshDistortionMode::LongitudinalWavePlane:
            {
                double gamma = rnd(1.0f, 1.5f);
                double halfPeriodDividedByCellDiag = rnd(1.5f, 3.0f);
                float PI_div2 = 1.0f * M_PI_2;
                double angle_rad = rnd(-PI_div2, PI_div2);
                mesh_generator::longitudinal::plane_waves::createLongitudinalTiltWaveGammaWarpMesh(
                    primeMesh, warpMesh, workFrameSize, meshGridSize, gamma, halfPeriodDividedByCellDiag,
                    mesh_nodes_move::WaveCallbackMeshPropagationAxis::axisX, angle_rad);
            }
            break;

        case MeshDistortionMode::LongitudinalWaveCylindrical:
            {
                double gamma = rnd(1.0f, 1.5f);
                double halfPeriodDividedByCellDiag = rnd(1.5f, 3.0f);
                float PI_div2 = 1.0f * M_PI_2;

                // center point
                cv::Point frame_center_point = {workFrameSize.width / 2, workFrameSize.height / 2};
                double scale_factor_center_point_pos = 0.7;
                int source_point_x_amplitude = static_cast<int>(scale_factor_center_point_pos * workFrameSize.width);
                int source_point_y_amplitude = static_cast<int>(scale_factor_center_point_pos * workFrameSize.height);

                int dx_source_point = rnd(-source_point_x_amplitude, source_point_x_amplitude);
                int dy_source_point = rnd(-source_point_y_amplitude, source_point_y_amplitude);

                cv::Point random_source_point = {frame_center_point.x + dx_source_point, frame_center_point.y + dy_source_point};
                
                mesh_generator::longitudinal::concentric_spherical_waves::createLongitudinalWaveGammaFromSourcePointConcentric(
                    primeMesh, warpMesh, workFrameSize, meshGridSize, random_source_point, gamma, halfPeriodDividedByCellDiag);
            }
            break;

        default:
            {
                throw std::runtime_error("Error: not supported current_warp_mode");
            }
        }
        if(MeshWarpApplicator::checkValidMesh(warpMesh))
        {
            success_create_valid_warp_mesh = true;
            std::cout << "      success, mode=" << current_warp_mode << std::endl;;
            break;
        }
        ++try_counter;
        std::cout << "      not success, mode =" << current_warp_mode << std::endl;
    }
    return success_create_valid_warp_mesh;
}

void getRandomPipelinedWarpMesh(
    const cv::Mat &primeMesh, 
    cv::Mat &warpMesh, 
    cv::Size workFrameSize, 
    cv::Size meshGridSize, 
    int pipeline_max_size, 
    int number_attempts_single_warp = 100,
    int number_attempts_pipeline_construct = 100)
{
    assert(pipeline_max_size >= 1);
    assert(number_attempts_single_warp >= 1);
    assert(number_attempts_pipeline_construct >= 1);


    
    int pipeline_size = rnd(1, pipeline_max_size);
    std::cout << "; pipeline_size=" << pipeline_size << std::endl;
    int try_counter = 0;
    bool success_construct_pipeline_valid_warp_mesh = false;
    bool status_generate_mesh = false;
    cv::Mat warpMeshTmp;

    // цикл по попыткам построения пайплайна искаженной сетки
    while(try_counter < number_attempts_pipeline_construct)
    {
        warpMeshTmp = primeMesh.clone();
        for(int pipe_stage = 0; pipe_stage < pipeline_size; ++pipe_stage)
        {
            status_generate_mesh = getRandomWarpMesh(
                warpMeshTmp, warpMeshTmp, 
                workFrameSize, meshGridSize, 
                number_attempts_single_warp);
            if(!status_generate_mesh)
            {
                break; // вне зависимости от этапа заново строится весь пайплайн
            }
            else
            {
                if(pipe_stage == pipeline_size - 1)
                {
                    success_construct_pipeline_valid_warp_mesh = true;
                    break;
                }
            }
        }
        if(success_construct_pipeline_valid_warp_mesh)
        {
            break;
        }
        ++try_counter;
    }
    assert(success_construct_pipeline_valid_warp_mesh);
    warpMesh = warpMeshTmp;
}
    
int main(int argc, char* argv[])
{
    int dataset_scale_factor = 5;
    int pipeline_max_size = 3; // 1 ... pipeline_max_size
    int warp_mesh_collection_size = 10;
    cv::Size process_frame_size = {416, 416};
    // cv::Size mesh_grid_size = {11,11};
    std::pair<int,int> mesh_grid_width_valid_range = {5, 15};
    std::pair<int,int> mesh_grid_height_valid_range = {5, 15};
    std::string path2srcDataset = "/home/vitaly/QtProg/Labeled_Image_Augmentation_Platform/data/test_labeled_images/";
    std::string path2dstDataset = "/home/vitaly/QtProg/Labeled_Image_Augmentation_Platform/data/dst_pipeline_results/";

    assert(dataset_scale_factor > 1);
    darknet_dataset::Dataset dataset_src;

    std::cout << "BEGIN setPath2DatasetDir" << std::endl;
    if(!dataset_src.setPath2DatasetDir(path2srcDataset))
    {
        throw std::runtime_error("Error: setPath2datasetDir return false");
    }
    std::cout << "END setPath2DatasetDir" << std::endl;

    std::cout << "BEGIN checkValid" << std::endl;
    if(!dataset_src.checkValid())
    {
        throw std::runtime_error("Error: checkValid return false");
    }
    std::cout << "END checkValid" << std::endl;

    std::vector<std::string> vecPaths2frames, vecPaths2annots;

    dataset_src.getValidDataset(vecPaths2frames, vecPaths2annots);

    std::string path2frame, path2annot;
    std::string dataset_sample_name;

    std::vector<std::shared_ptr<MeshWarpApplicator>> mesh_warp_applicator_collection;
    mesh_warp_applicator_collection.reserve(warp_mesh_collection_size);

    std::cout << "BEGIN generate warp mesh collection" << std::endl;
    int current_mesh_grid_width, current_mesh_grid_height;
    cv::Size current_meshGridSize;
    for(int i = 0; i < warp_mesh_collection_size; ++i)
    {
        current_mesh_grid_width = rnd(mesh_grid_width_valid_range.first, mesh_grid_width_valid_range.second);
        current_mesh_grid_height = rnd(mesh_grid_height_valid_range.first, mesh_grid_height_valid_range.second);
        current_meshGridSize = {current_mesh_grid_width, current_mesh_grid_height};
        std::cout << 
            "i=" << std::setw(4) << i + 1 << "/" << std::setw(4) << warp_mesh_collection_size << 
            "; current_meshGridSize = " << current_meshGridSize << std::endl; 
        cv::Mat srcMesh, warpMesh;
        srcMesh = mesh_generator::createPrimeMesh(process_frame_size, current_meshGridSize);
        getRandomPipelinedWarpMesh(srcMesh, warpMesh, process_frame_size, current_meshGridSize, pipeline_max_size);
        
        mesh_warp_applicator_collection.push_back(
            std::make_shared<MeshWarpApplicator>(srcMesh, warpMesh)
        );
    }
    std::cout << "END generate warp mesh collection" << std::endl;

    for(int i = 0; i <mesh_warp_applicator_collection.size(); ++i)
    {
        std::cout << "size" << mesh_warp_applicator_collection[i]->getMeshGridSize() << std::endl;
    }

    int current_warp_mesh_id;
    MarkedFrame marked_frame_src, marked_frame_dst;
    for(int i = 0; i < vecPaths2frames.size(); ++i)
    {
        path2frame = vecPaths2frames[i];
        path2annot = vecPaths2annots[i];

        std::cout << "i=" << std::setw(6) << i + 1 << "/" << std::setw(6) << vecPaths2frames.size() << "; path=" << path2frame << std::endl;
        marked_frame_src = MarkedFrame(path2frame, path2annot);
        if(!path_processing::getClearName(path2frame, dataset_sample_name))
        {
            throw std::runtime_error("Error path_processing::getClearName return false");
        }
        marked_frame_src.resize(process_frame_size);
        marked_frame_src.save(path2dstDataset, dataset_sample_name);

        for(int j = 0; j < dataset_scale_factor - 1; ++j)
        {
            std::cout << "  j=" << std::setw(3) << j + 1 << "/" << std::setw(3) << dataset_scale_factor << std::endl;

            current_warp_mesh_id = rnd(0, warp_mesh_collection_size - 1);
            std::cout << "current_warp_mesh_id = " << current_warp_mesh_id << std::endl;
            std::shared_ptr<MeshWarpApplicator> wma = mesh_warp_applicator_collection[current_warp_mesh_id];
            wma->apply(marked_frame_src, marked_frame_dst);
            drawMesh(marked_frame_dst.getFrameRef(), wma->getWarpMesh(), colors::blue, true);
            marked_frame_dst.save(path2dstDataset, dataset_sample_name + "_" + std::to_string(j));
        }
    }
    return 0;
}
