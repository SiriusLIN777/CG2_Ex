// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#define DEBUG_MODE
#define INFO_MODE

#include "Bone.h"

#include <cgv/math/transformations.h>
#include <cgv/math/inv.h>

#include "math_helper.h"
#include "debug_macros.h"

Bone::Bone()
	: parent(nullptr), length(0.0f), direction_in_world_space(0.0, 0.0, 0.0), translationTransforms(0)
{}

Bone::~Bone()
{
	for (auto bone : children)
		delete bone;
	children.clear();

	dofs.clear();

	for (auto dof : orientation)
		delete dof;
	orientation.clear();
}

void Bone::calculate_matrices()
{
	orientationTransformGlobalToLocal.identity();
	std::for_each(orientation.begin(), orientation.end(), [&](AtomicTransform* t) {
		orientationTransformGlobalToLocal = orientationTransformGlobalToLocal * t->calculate_matrix();
		//DEBUG("orientation t[" << t->get_index_in_amc() << "]: " << t->get_name() << ", vale:" << t->get_value() );
		//DEBUG("**** t: " << std::endl << t->calculate_matrix());
		
	});
	orientationTransformLocalToGlobal = cgv::math::inv(orientationTransformGlobalToLocal);
	//DEBUG("***********" << i << "times, " << get_name() << "-orientationTransformLocalToGlobal: " << std::endl << orientationTransformLocalToGlobal);
	////
	// Task 3.1: Implement matrix calculation
		
	////
	// Compute Orientation from preivious joint ot current.
	
	// 1. Get Parent's transform.
	orientationTransformPrevJointToCurrent.identity();
	if (get_parent()) { 
		//DEBUG(get_name() << " have a parent: " << parent->get_name() << " with orientation: " << std::endl << parent->orientationTransformLocalToGlobal);
		orientationTransformPrevJointToCurrent = parent->orientationTransformLocalToGlobal; 
	}

	// 2. Compute previous to current transform.
	orientationTransformPrevJointToCurrent = orientationTransformGlobalToLocal * orientationTransformPrevJointToCurrent;
	std::string bone_head = get_parent() ? parent->get_name() : "null";
	//DEBUG("****Orientation from " << bone_head << " to " << get_name() << std::endl << orientationTransformPrevJointToCurrent);

	////
	// Compute Transform from current joint ot Next.
	// 1. Get child's tansform (how solve 2 children with just one transform?)
	//DEBUG(get_name() << "num of children: " << children.size());
	//DEBUG("translationTransformGlobalToLocal" << translationTransformGlobalToLocal);
	//DEBUG("root: " << get_bone_local_root_position());
	// 2. Compute current to next transform 
	//translationTransformCurrentJointToNext = 
	
	//translationTransformCurrentJointToNext.identity();
	//// 遍历当前关节的所有子关节
	//for (int i = 0; i < childCount(); ++i) {
	//	// 获取当前子关节的平移矩阵
	//	const Mat4& childTranslation = child_at(i)->get_translation_transform_current_joint_to_next();

	//	// 将当前子关节的平移矩阵与整体平移矩阵相乘
	//	translationTransformCurrentJointToNext = translationTransformCurrentJointToNext * childTranslation;
	//}
	
	
	//DEBUG("----------" << get_name() << "-translationTransformCurrentJointToNext: " << std::endl << translationTransformCurrentJointToNext);

	////
	// Task 4.6: Implement matrix calculation (skinning)

}

Mat4 Bone::calculate_transform_prev_to_current_with_dofs()
{
	////
	// Task 3.1: Implement matrix calculation

	Mat4 t;
	return t;
}

Mat4 Bone::calculate_transform_prev_to_current_without_dofs()
{
	////
	// Task 3.1: Implement matrix calculation

	Mat4 t;
	return t;
}

void Bone::add_dof(AtomicTransform* dof)
{
	dof->set_index_in_amc((int)dofs.size());	
	if (dynamic_cast<AtomicTranslationTransform*>(dof))
	{
		dofs.push_front(std::shared_ptr<AtomicTransform>(dof));
		++translationTransforms;
	}
	else
		dofs.insert(dofs.begin() + translationTransforms, std::shared_ptr<AtomicTransform>(dof));
}

void Bone::set_name(const std::string& name) { this->name = name; }
const std::string& Bone::get_name() const { return name; }

void Bone::set_direction_in_world_space(const Vec3& direction) { this->direction_in_world_space = direction; }
const Vec3& Bone::get_direction_in_world_space() const { return direction_in_world_space; }

void Bone::set_length(float l) { this->length = l; }
float Bone::get_length() const { return length; }

void Bone::add_axis_rotation(AtomicRotationTransform* transform) { orientation.push_front(transform); }
void Bone::add_child(Bone* child)
{
	child->set_parent(this);
	children.push_back(child);
}
Bone* Bone::child_at(int i) const { return children[i]; }
int Bone::childCount() const { return (int)children.size(); }

void Bone::set_parent(Bone* parent)
{
	this->parent = parent;
}
Bone* Bone::get_parent() const { return parent; }

int Bone::dof_count() const { return (int)dofs.size(); }
std::shared_ptr<AtomicTransform> Bone::get_dof(int dofIndex) const { return dofs[dofIndex]; }

const Mat4& Bone::get_binding_pose_matrix() const
{
	return transformLocalToGlobal;
}

const Mat4& Bone::get_translation_transform_current_joint_to_next() const { return translationTransformCurrentJointToNext; }
const Mat4& Bone::get_orientation_transform_prev_joint_to_current() const { return orientationTransformPrevJointToCurrent; }

Vec4 Bone::get_bone_local_root_position() const { return Vec4(0, 0, 0, 1); }
Vec4 Bone::get_bone_local_tip_position() const { return translationTransformCurrentJointToNext * Vec4(0, 0, 0, 1); }
