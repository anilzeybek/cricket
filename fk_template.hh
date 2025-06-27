#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::panda
{
    static constexpr std::size_t dimension = {{n_q}};
    static constexpr std::size_t n_spheres = {{n_spheres}};

    using Configuration = FloatVector<dimension>;
    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_m_a{
        5.9342f,
        3.6652f,
        5.9342f,
        3.2289f,
        5.9342f,
        3.9095999999999997f,
        5.9342f};
    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_a_a{
        -2.9671f,
        -1.8326f,
        -2.9671f,
        -3.1416f,
        -2.9671f,
        -0.0873f,
        -2.9671f};

    const Configuration s_m(s_m_a);
    const Configuration s_a(s_a_a);

    inline void scale_configuration(Configuration &q) noexcept
    {
        q = q * s_m + s_a;
    }

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> d_m_a{
        0.1685147113342995f,
        0.2728364072901888f,
        0.1685147113342995f,
        0.30970299482796f,
        0.1685147113342995f,
        0.25578064252097404f,
        0.1685147113342995f};
    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> d_s_a{
        -2.9671f,
        -1.8326f,
        -2.9671f,
        -3.1416f,
        -2.9671f,
        -0.0873f,
        -2.9671f};

    const Configuration d_m(d_m_a);
    const Configuration d_s(d_s_a);

    inline void descale_configuration(Configuration &q) noexcept
    {
        q = (q - d_s) * d_m;
    }

    template <std::size_t rake>
    inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -2.9671f + (q[0] * 5.9342f);
        q[1] = -1.8326f + (q[1] * 3.6652f);
        q[2] = -2.9671f + (q[2] * 5.9342f);
        q[3] = -3.1416f + (q[3] * 3.2289f);
        q[4] = -2.9671f + (q[4] * 5.9342f);
        q[5] = -0.0873f + (q[5] * 3.9095999999999997f);
        q[6] = -2.9671f + (q[6] * 5.9342f);
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = 0.1685147113342995f * (q[0] - -2.9671f);
        q[1] = 0.2728364072901888f * (q[1] - -1.8326f);
        q[2] = 0.1685147113342995f * (q[2] - -2.9671f);
        q[3] = 0.30970299482796f * (q[3] - -3.1416f);
        q[4] = 0.1685147113342995f * (q[4] - -2.9671f);
        q[5] = 0.25578064252097404f * (q[5] - -0.0873f);
        q[6] = 0.1685147113342995f * (q[6] - -2.9671f);
    }

    inline static auto space_measure() noexcept -> float
    {
        return 878819.1112640093;
    }

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        return true;
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk_attachment(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        if (/*panda_link5 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2399, ADD_2400, ADD_2401, 0.173531, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
        }  // (1179, 1179)
        if (/*panda_rightfinger*/ sphere_environment_in_collision(
            environment, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_environment_in_collision(environment, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }

        set_attachment_pose(environment, ADD_969, ADD_970, ADD_971, SUB_1039, ADD_1050, SUB_1063, ADD_1074);
        if (/*attachment vs. panda_link0*/ attachment_sphere_collision<decltype(q[0])>(
            environment, 0.0, 0.0, 0.05, 0.08))
        {
            return false;
        }  // (1179, 1180)
        if (attachment_environment_collision(environment))
        {
            return false;
        }
        return true;
    }

    inline auto eefk(const std::array<float, 7> &q) noexcept -> std::array<float, 7>
    {
    }
}  // namespace vamp::robots::panda

// NOLINTEND(*-magic-numbers)
