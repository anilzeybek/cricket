#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::{{name}}
{
    static constexpr std::size_t dimension = {{n_q}};
    static constexpr std::size_t n_spheres = {{n_spheres}};

    using Configuration = FloatVector<dimension>;

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_m_a{
        {{bound_range}}
    };

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_a_a{
        {{bound_lower}}
    };

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> d_m_a{
        {{bound_descale}}
    };

    const Configuration s_m(s_m_a);
    const Configuration s_a(s_a_a);
    const Configuration d_m(d_m_a);

    inline void scale_configuration(Configuration & q) noexcept
    {
        q = q * s_m + s_a;
    }

    inline void descale_configuration(Configuration & q) noexcept
    {
        q = (q - s_a) * d_m;
    }

    template <std::size_t rake>
    inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        {% for index in range(n_q) %}
        q[{{index}}] = {{ at(bound_lower, index) }} + (q[{{index}}] * {{ at(bound_range, index) }});
        {% endfor %}
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        {% for index in range(n_q) %}
        q[{{index}}] = {{ at(bound_descale, index) }} * (q[{{index}}] - {{ at(bound_lower, index) }});
        {% endfor %}
    }

    inline static auto space_measure() noexcept -> float
    {
        return {{ measure }};
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
    }

    inline auto eefk(const std::array<float, 7> &x) noexcept -> std::array<float, 7>
    {
        float v[{{eefk_code_vars}}];
        float y[7];
        {{eefk_code}}

        return std::array<float, 7>(y, y + 7);
    }
}

// NOLINTEND(*-magic-numbers)
