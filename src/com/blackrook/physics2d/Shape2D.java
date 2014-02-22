/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

/**
 * Describes the common elements of all shapes in the physics system.
 * @author Matthew Tropiano
 */
public interface Shape2D
{
	/** Returns the distance between the farthest point and the origin. */
	public float getRadius();

	/** Returns the square distance between the farthest point and the origin. */
	public float getSquareRadius();

	/** 
	 * Returns a hint to the broadphase detector if this shape 
	 * should use radii for initial collision checking instead of AABBs.
	 * @return true if so, false if not.
	 */
	public boolean useRadius();

	/** 
	 * Returns the half width of this shape.
	 */
	public float getHalfWidth();

	/** 
	 * Returns the half height of this shape.
	 */
	public float getHalfHeight();

}
