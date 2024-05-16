
#include <cgv/math/fvec.h>
#include "distance_surface.h"

// ======================================================================================
//  Task 1.2: GENERAL HINTS
//
//  The super class skeleton of distance_surface has a protected member called ::edges,
//  which contains a list of all edges defined in the skeleton. Similarily, the super
//  super class knot_vector has a member called points, which contains a list of all
//  points used by the edges, which skeleton::edges indexes into.
//  Also make sure to check the header file of the distance_surface class for useful
//  members.
//
// ======================================================================================


template <typename T>
typename distance_surface<T>::vec_type distance_surface<T>::get_edge_distance_vector(size_t i, const pnt_type &p) const
{
	vec_type v;

	// Task 1.2: Compute the distance vector from edge i to p.

	// line AB edge_vector[i] = edges[i].second - edges[i].first
	// already defined in update_edge_precomputations()
	vec_type vec_AB = edge_vector[i];
	// vector AP
	vec_type vec_AP = p - edges[i].first;
	// vector BP
	vec_type vec_BP = p - edges[i].second;

	double dotAPAB = vec_AP.x() * vec_AB.x() + \
		vec_AP.y() * vec_AB.y() + \
		vec_AP.z() * vec_AB.z();
	double dotBPAB = vec_BP.x() * vec_AB.x() + \
		vec_BP.y() * vec_AB.y() + \
		vec_BP.z() * vec_AB.z();

	double squarelenAB = vec_AB.x() * vec_AB.x() + \
		vec_AB.y() * vec_AB.y() + \
		vec_AB.z() * vec_AB.z();

	double t = dotAPAB/ squarelenAB;

	if (t < 0) {
		v = p - edges[i].first;
	}
	else if (t >= 1) {
		v = p - edges[i].second;
	}
	else {
		v.x() = p.x() - (edges[i].first + t * edge_vector[i].x());
		v.y() = p.y() - (edges[i].first + t * edge_vector[i].y());
		v.z() = p.z() - (edges[i].first + t * edge_vector[i].z());
	}

	/*
	double dot_product = point_vec.x() * edge_vector_inv_length[i].x() + \
		point_vec.y() * edge_vector_inv_length[i].y() + \
		point_vec.z() * edge_vector_inv_length[i].z();
	double line_magnitude = edge_vector_inv_length[i].x() * edge_vector_inv_length[i].x() + \
		edge_vector_inv_length[i].y() * edge_vector_inv_length[i].y() + \
		edge_vector_inv_length[i].z() * edge_vector_inv_length[i].z();
	
	// This function may consider all edges as rays.
	// It may provide wrong values if the edge is bounded in two points.
	vec_type proj_point;
	proj_point.x() = edges[i].first.x() + (dot_product / line_magnitude) * edge_vector_inv_length[i].x();
	proj_point.y() = edges[i].first.y() + (dot_product / line_magnitude) * edge_vector_inv_length[i].y();
	proj_point.z() = edges[i].first.z() + (dot_product / line_magnitude) * edge_vector_inv_length[i].z();

	v = p - proj_point;
	*/

	return v;
}

template <typename T>
double distance_surface<T>::get_min_distance_vector (const pnt_type &p, vec_type& v) const
{
	double min_dist = std::numeric_limits<double>::infinity();


	// Task 1.2: Compute the minimum distance from the skeleton to p, and report the
	//           corresponding distance vector in v.

	for (int i = 0; i < edge_vector.size(); i++) {
		vec_type current_v = get_edge_distance_vector(i,p);
		double current_d = sqrt(current_v.x() * current_v.x() + \
			current_v.y() * current_v.y() + \
			current_v.z() * current_v.z());
		if (current_d < min_dist) {
			min_dist = current_d;
			v = current_v;
		}
	}

	return min_dist;
}

template <typename T>
T distance_surface<T>::evaluate(const pnt_type& p) const
{
	double f_p = std::numeric_limits<double>::infinity();

	// Task 1.2: Evaluate the distance surface function at p.
	vec_type dummy_output;
	f_p = get_min_distance_vector(p, dummy_output) - r;



	return f_p;
}

template <typename T>
typename distance_surface<T>::vec_type distance_surface<T>::evaluate_gradient(const pnt_type& p) const
{
	vec_type grad_f_p(0, 0, 0);

	// Task 1.2: Return the gradient of the distance surface function at p.
	vec_type dummy_output;
	double val = get_min_distance_vector(p, dummy_output);

	grad_f_p.x() = 1 / val * p.x();
	grad_f_p.y() = 1 / val * p.y();
	grad_f_p.z() = 1 / val * p.z();

	return grad_f_p;
}

/// update helper variables for edge i
template <typename T>
void distance_surface<T>::update_edge_precomputations(size_t ei)
{
	edge_vector[ei] =
		  (knot_vector<T>::points)[(skeleton<T>::edges)[ei].second]
		- (knot_vector<T>::points)[(skeleton<T>::edges)[ei].first];
	edge_vector_inv_length[ei] = (T(1)/ edge_vector[ei].sqr_length()) * edge_vector[ei];
}

/// construct distance surface
template <typename T>
distance_surface<T>::distance_surface()
{
	r=0.5;
	gui_title_added = false;
}
/// reflect members to expose them to serialization
template <typename T>
bool distance_surface<T>::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return
		skeleton<T>::self_reflect(rh) &&
		rh.reflect_member("r", r);
}

template <typename T>
void distance_surface<T>::append_edge_callback(size_t ei)
{
	edge_vector.push_back(vec_type(0,0,0));
	edge_vector_inv_length.push_back(vec_type(0,0,0));
	update_edge_precomputations(ei);
}
template <typename T>
void distance_surface<T>::edge_changed_callback(size_t ei)
{
	update_edge_precomputations(ei);
}
template <typename T>
void distance_surface<T>::position_changed_callback(size_t pi)
{
	for (unsigned ei=0; ei<(skeleton<T>::edges).size(); ei++) 
		if ((skeleton<T>::edges)[ei].first == pi || (skeleton<T>::edges)[ei].second == pi)
			update_edge_precomputations(ei);
}

template <typename T>
void distance_surface<T>::create_gui()
{
	if (!gui_title_added) {
		provider::add_view("distance surface", named::name)->set("color",0xFF8888);
		gui_title_added = true;
	}

	provider::add_member_control(this, "radius", r, "value_slider", "min=0;max=5;log=true;ticks=true");

	skeleton<T>::create_gui();
}

scene_factory_registration<distance_surface<double> > sfr_distance_surface("distance_surface;D");
