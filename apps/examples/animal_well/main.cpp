// Port of phyverse semi_implicit_3d.cpp case 107 "animal pool"
//
// Mesh files required in the asset directories:
//   assets/sim_data/trimesh/pool.obj         -- fixed container
//   assets/sim_data/tetmesh/animal_well.msh  -- deformable FEM body
//
// animal_well.msh must be converted from the TetGen-format
// animal_well.1.mesh produced by the phyverse case 107 setup.

#include <app/asset_dir.h>
#include <uipc/uipc.h>
#include <uipc/builtin/constants.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/stable_neo_hookean.h>

int main()
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    std::string trimesh_dir{AssetDir::trimesh_path()};
    auto this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    std::string contact_constitution = "al-ipc";

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.81, 0};
    config["dt"]                            = 0.01;
    config["linear_system"]["tol_rate"]     = 1e-4;
    config["line_search"]["max_iter"]       = 256;
    config["contact"]["enable"]             = true;
    config["contact"]["d_hat"]              = 0.002;
    config["contact"]["friction"]["enable"] = false;
    config["contact"]["constitution"]       = contact_constitution;
    if(contact_constitution == "al-ipc")
    {
        config["newton"]["min_iter"]                 = 2;
        config["contact"]["al-ipc"]["toi_threshold"] = 0.001;
        config["contact"]["al-ipc"]["decay_factor"]  = 0.9;
        config["newton"]["velocity_tol"]             = 1.0;
    }

    Scene scene{config};
    {
        StableNeoHookean snh;
        NeoHookeanShell  nhs;

        scene.contact_tabular().default_model(0.0, 1.0_GPa);
        auto default_element = scene.contact_tabular().default_element();

        SimplicialComplexIO io;

        // Pool -- fixed FEM shell (NeoHookeanShell, all vertices is_fixed=1)
        auto pool = io.read(fmt::format("{}pool.obj", trimesh_dir));
        label_surface(pool);
        nhs.apply_to(pool, ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49));
        default_element.apply_to(pool);
        {
            auto is_fixed = view(*pool.vertices().find<IndexT>(builtin::is_fixed));
            std::fill(is_fixed.begin(), is_fixed.end(), 1);
        }

        auto pool_obj = scene.objects().create("pool");
        pool_obj->geometries().create(pool);

        // Animal well -- deformable Neo-Hookean body (E=5e5 Pa, nu=0.3, rho=1e3 kg/m^3)
        auto animal = io.read(fmt::format("{}animal_well.msh", tetmesh_dir));
        label_surface(animal);
        label_triangle_orient(animal);
        snh.apply_to(animal, ElasticModuli::youngs_poisson(5e5, 0.3), 1e3);
        default_element.apply_to(animal);

        auto animal_obj = scene.objects().create("animal_well");
        animal_obj->geometries().create(animal);
    }

    world.init(scene);
    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));
    world.dump();

    while(world.frame() < 300)
    {
        world.advance();
        world.retrieve();
        world.dump();
        sio.write_surface(
            fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
        fmt::println("frame: {}", world.frame());
    }
}
