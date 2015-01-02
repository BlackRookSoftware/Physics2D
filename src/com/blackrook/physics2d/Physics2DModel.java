/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

import com.blackrook.commons.math.Tuple2D;
import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.commons.math.geometry.Vect2D;

/**
 * This model the defines how objects intersect with each other.  
 * @author Matthew Tropiano
 */
public interface Physics2DModel<T extends Object>
{
	/** 
	 * Returns all objects in this model.
	 */
	public Iterable<T> getAllObjects();

	/**
	 * Gets an object's centerpoint.
	 */
	public void getObjectCollisionCenter(T object, Point2D center);
	
	/**
	 * Gets an object's half-widths for its BOUNDING area.
	 * This area is used for broadphase collision detection, and may not represent the object's actual collision zone!
	 */
	public void getObjectCollisionHalfWidths(T object, Tuple2D halfwidths);
	
	/**
	 * Gets the velocity of this body along the X-axis.
	 * This also influences collision along separating axes and incident vectors.
	 * The cumulative area is used for broadphase collision detection, and may not represent the object's actual collision zone!
	 */
	public void getObjectCollisionVelocity(T object, Vect2D velocity);

	/**
	 * Gets if an object is in motion, usually because its velocity is nonzero.
	 */
	public boolean isObjectInMotion(T object);

	/**
	 * Gets the object's squared radius, eliminating a 
	 * potentially expensive square root call.
	 * This area is used for broadphase collision detection, and may not represent the object's actual collision zone!
	 * <p>
	 * Since this could be an expensive call, this 
	 * is not always used - it is used if the object's useRadius() 
	 * function returns true, which leaves it in the hands of the implementor.
	 */
	public double getObjectCollisionSquaredRadius(T object);

	/**
	 * Gets the rotation of this body around the Z-axis.
	 * This also influences collision along separating axes and incident vectors.
	 * This may not have an impact on different shapes like Boxes and Circles.
	 * @return the rotation from the base positioning in degrees.
	 */
	public double getObjectCollisionRotationZ(T object);

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
	public long getObjectCollisionGroupMask(T object);

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
	public long getObjectCollisionTargetMask(T object);

	/** 
	 * Returns the internal shape to use for this body.
	 * If this returns null, this object is eliminated from collision detection. 
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public Shape2D getObjectCollisionShape(T object);

	/**
	 * Returns true if this object is to be excluded from collision detection entirely,
	 * If false, it is tested like any other object.
	 * <p>This is one of the many ways to exclude an object from collision detection.
	 */
	public boolean isObjectExcludedFromCollision(T object);
	
}
