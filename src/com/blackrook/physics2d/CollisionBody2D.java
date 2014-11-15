/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

import com.blackrook.commons.spatialhash.SpatialHash2D;
import com.blackrook.commons.spatialhash.SpatialHashable;

/**
 * Defines an interface for all collidable objects usable by the PhysicsSystem.
 * Most of the methods defined herein are meant to be compatible with {@link SpatialHash2D}
 * in the Commons project, as {@link SpatialHash2D} is used for broad-phase detection.
 * @author Matthew Tropiano
 */
public interface CollisionBody2D extends SpatialHashable
{
	/** 
	 * Returns the internal shape to use for this body.
	 * If this returns null, this object is eliminated from collision detection. 
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public Shape2D getCollisionShape();

	/**
	 * Returns the group mask of this object (what my type is).
	 * The group mask is a bitmask describing the type of object
	 * this is to the physics system, arbitrarily defined by the
	 * implementor. The purpose of this, when used in conjunction with
	 * the target mask, is to eliminate potential collisions based on
	 * an object's type or characteristics.
	 * <p>A mask of -1 (all bits set) means that this is every type.
	 * <br>A mask of 0 is no type, and collides with nothing.
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public long getCollisionGroupMask();
	
	/**
	 * Returns the target mask of this object (what can I collide with?).
	 * <p>The target mask is a bitmask describing the type of object
	 * this is to the physics system, arbitrarily defined by the
	 * implementor. The purpose of this, when used in conjunction with
	 * the group mask, is to eliminate potential collisions based on
	 * an object's type or characteristics.
	 * <p>A mask of -1 (all bits set) can collide with everything.
	 * <br>A mask of 0 collides with nothing.
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public long getCollisionTargetMask();
	
	/**
	 * Returns true if this object is to be excluded from collision detection entirely,
	 * If false, it is tested like any other object.
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public boolean isExcludedFromCollision();
	
	/**
	 * Gets the object's squared radius, eliminating a 
	 * potentially expensive square root call.
	 * <p>
	 * Since this could be an expensive call, this 
	 * is not always used - it is used if the object's useRadius() 
	 * function returns true, which leaves it in the hands of the implementor.
	 */
	public double getCollisionSquaredRadius();
	
	/**
	 * Gets the velocity of this body along the X-axis.
	 * This also influences collision along separating axes and incident vectors.
	 * @return the velocity in units along the X-axis.
	 */
	public double getCollisionVelocityX();

	/**
	 * Gets the velocity of this body along the Y-axis.
	 * This also influences collision along separating axes and incident vectors.
	 * @return the velocity in units along the Y-axis.
	 */
	public double getCollisionVelocityY();

	/**
	 * Gets the rotation of this body around the Z-axis.
	 * This also influences collision along separating axes and incident vectors.
	 * @return the rotation from the base positioning in degrees.
	 */
	public double getCollisionRotationZ();
	
}

