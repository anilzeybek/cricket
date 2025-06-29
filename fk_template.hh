#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

template <typename DataT>
inline constexpr auto sin(const DataT &v) -> DataT
{
    return v.sin();
}

template <typename DataT>
inline constexpr auto cos(const DataT &v) -> DataT
{
    return v.cos();
}

template <typename DataT>
inline constexpr auto sqrt(const DataT &v) -> DataT
{
    return v.sqrt();
}

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::{{name}}
{
    static constexpr std::size_t dimension = {{n_q}};
    static constexpr std::size_t n_spheres = {{n_spheres}};
    static constexpr float min_radius = {{min_radius}};
    static constexpr float max_radius = {{max_radius}};
    static constexpr std::array<std::string, dimension> names = {"{{join(joint_names, "\", \"")}}"};
    static constexpr std::string end_effector = "{{end_effector}}";

    using Configuration = FloatVector<dimension>;

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_m_a{
        {{join(bound_range, ", ")}}
    };

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> s_a_a{
        {{join(bound_lower, ", ")}}
    };

    alignas(Configuration::S::Alignment) constexpr std::array<float, dimension> d_m_a{
        {{join(bound_descale, ", ")}}
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
        {% for index in range(n_q) %} q[{{index}}] = {{ at(bound_lower, index) }} + (q[{{index}}] * {{ at(bound_range, index) }});
        {% endfor %}
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        {% for index in range(n_q) %} q[{{index}}] = {{ at(bound_descale, index) }} * (q[{{index}}] - {{ at(bound_lower, index) }});
        {% endfor %}
    }

    inline static auto space_measure() noexcept -> float
    {
        return {{measure}};
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
    inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, {{spherefk_code_vars}}> v;
        std::array<FloatVector<rake, 1>, {{spherefk_code_output}}> y;

        {{spherefk_code}}

        for (auto i = 0U; i < {{n_spheres}}; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    template <std::size_t rake>
        inline bool interleaved_sphere_fk(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, {{ccfk_code_vars}}> v;
        std::array<FloatVector<rake, 1>, {{ccfk_code_output}}> y;

        {{ccfk_code}}

        {% for i in range(length(links_with_geometry)) %}
        {% set array_index = length(links_with_geometry) - i - 1 %}
        {% set link_index = at(links_with_geometry, array_index) %}
        {% set link_spheres = at(per_link_spheres, link_index) %}

        // {{ at(link_names, link_index) }}
        if (sphere_environment_in_collision(environment,
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 3]
                ))
        {
            {% for j in range(length(link_spheres)) %}
            {% set sphere_index = at(link_spheres, j) %}
            if (sphere_environment_in_collision(environment,
                                                y[{{ sphere_index }} * 4 + 0],
                                                y[{{ sphere_index }} * 4 + 1],
                                                y[{{ sphere_index }} * 4 + 2],
                                                y[{{ sphere_index }} * 4 + 3]))
                {
                    return false;
                }
            {% endfor %}
        }

        {% endfor %}

        {% for i in range(length(allowed_link_pairs)) %}
        {% set pair = at(allowed_link_pairs, i) %}
        {% set link_1_index = at(pair, 0) %}
        {% set link_2_index = at(pair, 1) %}
        {% set link_1_bs = at(bounding_sphere_index, link_1_index) %}
        {% set link_2_bs = at(bounding_sphere_index, link_2_index) %}
        {% set link_1_spheres = at(per_link_spheres, link_1_index) %}
        {% set link_2_spheres = at(per_link_spheres, link_2_index) %}

        // {{ at(link_names, link_1_index) }} vs. {{ at(link_names, link_2_index) }}
        if (sphere_sphere_self_collision<decltype(x[0])>(
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 3],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 3]))
        {
            {% for j in range(length(link_1_spheres)) %}
            {% for k in range(length(link_2_spheres)) %}

            {% set sphere_1_index = at(link_1_spheres, j) %}
            {% set sphere_2_index = at(link_2_spheres, k) %}

            if (sphere_sphere_self_collision<decltype(x[0])>(
                                                y[{{ sphere_1_index }} * 4 + 0],
                                                y[{{ sphere_1_index }} * 4 + 1],
                                                y[{{ sphere_1_index }} * 4 + 2],
                                                y[{{ sphere_1_index }} * 4 + 3],
                                                y[{{ sphere_2_index }} * 4 + 0],
                                                y[{{ sphere_2_index }} * 4 + 1],
                                                y[{{ sphere_2_index }} * 4 + 2],
                                                y[{{ sphere_2_index }} * 4 + 3]))
            {
                return false;
            }

            {% endfor %}
            {% endfor %}
        }
        {% endfor %}

        return true;
    }

    template <std::size_t rake>
        inline bool interleaved_sphere_fk_attachment(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, {{ccfkee_code_vars}}> v;
        std::array<FloatVector<rake, 1>, {{ccfkee_code_output}}> y;

        {{ccfkee_code}}

        {% for i in range(length(links_with_geometry)) %}
        {% set array_index = length(links_with_geometry) - i - 1 %}
        {% set link_index = at(links_with_geometry, array_index) %}
        {% set link_spheres = at(per_link_spheres, link_index) %}

        // {{ at(link_names, link_index) }}
        if (sphere_environment_in_collision(environment,
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ array_index }}) * 4 + 3]
                ))
        {
            {% for j in range(length(link_spheres)) %}
            {% set sphere_index = at(link_spheres, j) %}
            if (sphere_environment_in_collision(environment,
                                                y[{{ sphere_index }} * 4 + 0],
                                                y[{{ sphere_index }} * 4 + 1],
                                                y[{{ sphere_index }} * 4 + 2],
                                                y[{{ sphere_index }} * 4 + 3]))
                {
                    return false;
                }
            {% endfor %}
        }

        {% endfor %}

        {% for i in range(length(allowed_link_pairs)) %}
        {% set pair = at(allowed_link_pairs, i) %}
        {% set link_1_index = at(pair, 0) %}
        {% set link_2_index = at(pair, 1) %}
        {% set link_1_bs = at(bounding_sphere_index, link_1_index) %}
        {% set link_2_bs = at(bounding_sphere_index, link_2_index) %}
        {% set link_1_spheres = at(per_link_spheres, link_1_index) %}
        {% set link_2_spheres = at(per_link_spheres, link_2_index) %}

        // {{ at(link_names, link_1_index) }} vs. {{ at(link_names, link_2_index) }}
        if (sphere_sphere_self_collision<decltype(x[0])>(
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ link_1_bs }}) * 4 + 3],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 0],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 1],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 2],
                                            y[({{ n_spheres }} + {{ link_2_bs }}) * 4 + 3]))
        {
            {% for j in range(length(link_1_spheres)) %}
            {% for k in range(length(link_2_spheres)) %}

            {% set sphere_1_index = at(link_1_spheres, j) %}
            {% set sphere_2_index = at(link_2_spheres, k) %}

            if (sphere_sphere_self_collision<decltype(x[0])>(
                                                y[{{ sphere_1_index }} * 4 + 0],
                                                y[{{ sphere_1_index }} * 4 + 1],
                                                y[{{ sphere_1_index }} * 4 + 2],
                                                y[{{ sphere_1_index }} * 4 + 3],
                                                y[{{ sphere_2_index }} * 4 + 0],
                                                y[{{ sphere_2_index }} * 4 + 1],
                                                y[{{ sphere_2_index }} * 4 + 2],
                                                y[{{ sphere_2_index }} * 4 + 3]))
            {
                return false;
            }

            {% endfor %}
            {% endfor %}
        }

        {% endfor %}

        // Attaching at {{ end_effector }}
        set_attachment_pose(environment,
                            y[{{ccfkee_code_output}} - 7],
                            y[{{ccfkee_code_output}} - 6],
                            y[{{ccfkee_code_output}} - 5],
                            y[{{ccfkee_code_output}} - 4],
                            y[{{ccfkee_code_output}} - 3],
                            y[{{ccfkee_code_output}} - 2],
                            y[{{ccfkee_code_output}} - 1]
            );

        {% for i in range(length(end_effector_collisions)) %}
        {% set link_index = at(end_effector_collisions, i) %}
        {% set link_bs = at(bounding_sphere_index, link_index) %}
        {% set link_spheres = at(per_link_spheres, link_index) %}

        // Attachment vs. {{ at(link_names, link_index )}}
        if (attachment_sphere_collision<decltype(q[0])>(
                environment,
                y[({{ n_spheres }} + {{ link_bs }}) * 4 + 0],
                y[({{ n_spheres }} + {{ link_bs }}) * 4 + 1],
                y[({{ n_spheres }} + {{ link_bs }}) * 4 + 2],
                y[({{ n_spheres }} + {{ link_bs }}) * 4 + 3]))
        {
            {% for j in range(length(link_spheres)) %}
            {% set sphere_index = at(link_spheres, j) %}
            if (attachment_sphere_collision<decltype(q[0])>(
                    environment,
                    y[{{ sphere_index }} * 4 + 0],
                    y[{{ sphere_index }} * 4 + 1],
                    y[{{ sphere_index }} * 4 + 2],
                    y[{{ sphere_index }} * 4 + 3]))
                {
                    return false;
                }
            {% endfor %}
        }

        {% endfor %}

        return true;
    }

    inline auto eefk(const std::array<float, {{n_q}}> &x) noexcept -> std::array<float, 7>
    {
        std::array<float, {{eefk_code_vars}}> v;
        std::array<float, 7> y;

        {{eefk_code}}

        return y;
    }
}

// NOLINTEND(*-magic-numbers)
