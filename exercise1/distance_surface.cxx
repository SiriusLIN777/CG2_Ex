#define DEBUG_MODE

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


// ======================================================================================
//  Compute projection on the edge.
//		1. vector_AB = vertice_B (x1,y1,z1) - vertice_A(x2,y2,z2) = (x2 - x1, y2 - y1, z2 - z1);
//		2. plane_projection(PP) = P - (vector_AP * vector_AB) / |vector_AB|^2 * vector_AB;
//		3. line_projection(LP):
//			a. distance_PP_A = |PP - A|
//			b. line_projection = A + (distance_PP_A / |vector_AB|) * vector_AB
// ======================================================================================
template <typename T>
typename distance_surface<T>::vec_type distance_surface<T>::get_edge_distance_vector(size_t i, const pnt_type &p) const
{
	vec_type v;

	// Task 1.2: Compute the distance vector from edge i to p.
	vec_type vec_AB = edge_vector[i];
	vec_type vec_BA = edge_vector[i] * -1;
	vec_type vec_AP = p - points[edges[i].first];
	vec_type vec_BP = p - points[edges[i].second];
	vec_type vec_LP; // L is the projection point on edgei of p.
	/*
	if ((vec_AB.x() * vec_AP.x() + vec_AB.y() * vec_AP.y() + vec_AB.z() * vec_AP.z()) >= 0 &&
		vec_AB.length() >= vec_AP.length()) // Projection of p is on line segment AB
	{

		//vec_type vec_PP = p - ((vec_AP * vec_AB) / vec_AB.sqr_length()) * vec_AB;	// P's Projection on line-AB's Plane
		//double d = (vec_PP - points[edges[i].first]).length();	// Distance from PP to A
		//vec_type vec_PL = points[edges[i].first] + (d / vec_AB.length()) * vec_AB;	// P's Projection on line AB 
		double t = (vec_AB.x() * vec_AP.x() + vec_AB.y() * vec_AP.y() + vec_AB.z() * vec_AP.z()) / vec_AB.sqr_length();
		vec_PL = points[edges[i].first] + vec_AB * t;
	}
	else if ((vec_AB.x() * vec_AP.x() + vec_AB.y() * vec_AP.y() + vec_AB.z() * vec_AP.z()) < 0 &&
		vec_AP.length() <= r) // Projection of p is out line segment AB
	{
		vec_PL = points[edges[i].first];
	}
	else if ((vec_BA.x() * vec_BP.x() + vec_BA.y() * vec_BP.y() + vec_BA.z() * vec_BP.z()) > 0 &&
		vec_BP.length() <= r)
	{
		vec_PL = points[edges[i].second];
	}
	*/

	double t = (vec_AB.x() * vec_AP.x() + vec_AB.y() * vec_AP.y() + vec_AB.z() * vec_AP.z()) / vec_AB.sqr_length();
	//vec_PL_P = points[edges[i].first] + vec_AB * t;
	if (t >= 0 && t <= 1)
	{
		vec_LP = vec_AP - t * vec_AB;
	}
	else if (t < 0)
	{
		vec_LP = vec_AP;
	}
	else if (t > 1)
	{
		vec_LP = vec_BP;
	}

	v = vec_LP;

	//DEBUG("edge AB:" << points[edges[i].first] << " -- " << points[edges[i].second]);
	//DEBUG("vec AB cal: " << points[edges[i].second] - points[edges[i].first]);
	//DEBUG("vec_AB: " << vec_AB);
	//DEBUG("pp on edge:" << vec_PL);

	return v; // return the vector Projection_on_Line(PL)->p
}

template <typename T>
bool distance_surface<T>::is_vertice(size_t i, const pnt_type& p, vec_type& vec_IP) const
{
	bool is_vertice = false;

	
	is_vertice = 
		(vec_IP == (p - points[edges[i].first])) || (vec_IP == (p - points[edges[i].second])) ?
		true : false;
	

	return is_vertice;
}

template <typename T>
double distance_surface<T>::get_min_distance_vector (const pnt_type &p, vec_type& v) const
{
	double min_dist = std::numeric_limits<double>::infinity();

	// Task 1.2: Compute the minimum distance from the skeleton to p, and report the
	//           corresponding distance vector in v.

	/*
	for (int i = 0; i < edges.size(); i++)
	{
		//DEBUG("edge at: " << i);
		vec_type vec_IP = get_edge_distance_vector(i, p);
		if (is_vertice(i, p, vec_IP))
		{
			min_dist = vec_IP.length();
		}
		else
		{
			double dist = vec_IP.length();
			//DEBUG("edge_distance: " << dist);
			if (min_dist > dist)
			{
				min_dist = dist;
				v = vec_IP;
			}
		}
		
	}
	*/
	
	/*
	for (int i = 0; i < edges.size(); i++)
	{
		vec_type vec_PL = get_edge_distance_vector(i, p);
		vec_type vec_AP = p - points[edges[i].first];
		vec_type vec_BP = p - points[edges[i].second];
		
		double dist;	// dist from skeleton to p.
		vec_type vec_IP;	// vector from skeleton i to p.

		if (vec_AP.length() < vec_BP.length()) // P is nearer to A.
		{
			vec_type vec_A_PL = p - vec_PL - points[edges[i].first];
			vec_type vec_AB = points[edges[i].second] - points[edges[i].first];

			if ((vec_AP.x() * vec_AB.x() + vec_AP.y() * vec_AB.y() + vec_AP.z() * vec_AB.z())
				< 0) // Projection falls out of line segment AB;
			{
				dist = vec_AP.length();
				v = vec_AP;
			}
			else // Projection falls on line segment AB.
			{
				dist = vec_PL.length();
				v = vec_PL;
			}
		}
		else if (vec_BP.length() < vec_AP.length())
		{
			vec_type vec_B_PL = p - vec_PL - points[edges[i].second];
			vec_type vec_BA = points[edges[i].first] - points[edges[i].second];

			if ((vec_BP.x() * vec_BA.x() + vec_AP.y() * vec_BA.y() + vec_AP.z() * vec_BA.z())
				< 0) // Projection falls out of line segment AB.
			{
				dist = vec_BP.length();
				v = vec_BP;
			}
			else // Projection falls on line segment AB.
			{
				dist = vec_PL.length();
				v = vec_PL;
			}
		}
		else
		{
			dist = vec_PL.length();
			v = vec_PL;
		}

		
		//DEBUG("edge_distance: " << dist);
		if (min_dist > dist)
		{
			min_dist = dist;
			v = vec_IP;
		}

	}
	*/

	/*
	for (int i = 0; i < edges.size(); i++)
	{
		vec_type vec_PL_P = get_edge_distance_vector(i, p);
		vec_type vec_AP = p - points[edges[i].first];
		vec_type vec_BP = p - points[edges[i].second];

		double dist;	// dist from skeleton to p.
		vec_type vec_IP;	// vector from skeleton i to p.

		vec_type projection_L = p - vec_PL_P;

		vec_type vec_AL = projection_L - points[edges[i].first];
		vec_type vec_BL = projection_L - points[edges[i].second];

		if ((vec_AL.x() * vec_BL.x() + vec_AL.y() * vec_BL.y() + vec_AL.z() * vec_BL.z()) < 0) // L falls out line segment AB.
		{
			// L on A side: vec_AP, L on B side: vec_BP
			dist = vec_AL.length() < vec_BL.length() ? vec_AP.length() : vec_BP.length();
			vec_IP = vec_AL.length() < vec_BL.length() ? vec_AP : vec_BP;
		}
		else	// L falls on line segment AB.
		{
			dist = vec_PL_P.length();
			vec_IP = vec_PL_P;
		}

		if (min_dist > dist)
		{
			min_dist = dist;
			v = vec_IP;
			//DEBUG("p:" << p << ", min dist updated at:" << i <<", with dist:" << min_dist);
		}

	}
	*/

	for (int i = 0; i < edges.size(); i++)
	{
		vec_type vec_IP = get_edge_distance_vector(i, p);
		double dist = vec_IP.length();
		//DEBUG("edge_distance: " << dist);
		if (min_dist > dist)
		{
			min_dist = dist;
			v = vec_IP;
		}
	
	}

	//DEBUG("min edge idx: " << idx);
	//if(min_dist == r) DEBUG("min edge distance vec: " << v);

	return min_dist;
}

template <typename T>
T distance_surface<T>::evaluate(const pnt_type& p) const
{
	double f_p = std::numeric_limits<double>::infinity();

	// Task 1.2: Evaluate the distance surface function at p.
	vec_type v(0,0,0);
	
	f_p = get_min_distance_vector(p, v) - r;

	//DEBUG("f_p: " << f_p);

	return f_p;
}

template <typename T>
typename distance_surface<T>::vec_type distance_surface<T>::evaluate_gradient(const pnt_type& p) const
{
	vec_type grad_f_p(0, 0, 0);

	// Task 1.2: Return the gradient of the distance surface function at p.

	vec_type v(0, 0, 0);
	get_min_distance_vector(p, v) - r;
	grad_f_p = v;

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
