/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.commons.math.geometry.Vect2D;

/**
 * Contains the metadata necessary for holding information 
 * on a collision between two bodies.
 * @author Matthew Tropiano
 */
public class Collision2D<T extends CollisionBody2D>
{
	public static enum Method
	{
		CIRCLE_TO_CIRCLE,
		CIRCLE_TO_BOX,
		BOX_TO_CIRCLE,
		BOX_TO_BOX,
		SEPARATING_AXIS;
	}
	
	/** Reference to the source body (collider). */
	public T source;
	/** Reference to the target body (collidee). */
	public T target;
	/** Incident vector (overlap). */
	public final Vect2D incidentVector;
	/** Incident point (point of collision). */
	public final Point2D incidentPoint;
	/** Calculation time in nanoseconds. */
	public long calcNanos;
	/** Number of separating axes tested. */
	public int axisCount;
	/** Detection method. */
	public Method method;
	
	public Collision2D()
	{
		source = null;
		target = null;
		incidentVector = new Vect2D();
		incidentPoint = new Point2D();
		calcNanos = 0L;
		axisCount = 0;
		method = null;
	}

	/**
	 * Flips the point and vector of incidence.
	 */
	public void flipIncident()
	{
		incidentPoint.add(incidentVector);
		incidentVector.scale(-1, -1);
	}
	
	@Override
	public String toString()
	{
		return "Source: "+source.toString()+" Target: "+target.toString()+
			" IV:"+incidentVector.toString()+" IP:"+incidentPoint+" T: "+calcNanos+"ns";
	}

}
