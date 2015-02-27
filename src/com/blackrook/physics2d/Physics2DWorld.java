/*******************************************************************************
 * Copyright (c) 2014 - 2015 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

/**
 * This world model defines how to figure out broadphase object culling and interaction. 
 * @author Matthew Tropiano
 */
public interface Physics2DWorld<T extends Object>
{
	/** 
	 * Returns all objects in this model.
	 */
	public Iterable<T> getAllObjects();

	/**
	 * Returns a set of objects that can potentially intersect with each other.
	 * How this potential set is determined is up to the implementation, but should return
	 * a limited amount of objects relatively quickly. The objects returned need to be tested
	 * for a collision - not all
	 * @param object the object to test for intersecting.
	 * @param outArray the output array.
	 * @param offset the starting offset into the array. 
	 * @return the amount of objects returned. 
	 */
	public int getPotentialObjectIntersections(T object, Object[] outArray, int offset);

	/**
	 * Returns a set of objects that can potentially intersect with a straight line.
	 * @param object the object to test for intersecting.
	 * @param outArray the output array.
	 * @param offset the starting offset into the array. 
	 * @return the amount of objects returned. 
	 */
	public int getPotentialRaycastIntersections(T object, Object[] outArray, int offset);

}
