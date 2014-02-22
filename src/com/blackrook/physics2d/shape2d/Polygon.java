/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.shape2d;

import java.util.Arrays;

import com.blackrook.commons.math.geometry.Point2F;
import com.blackrook.commons.math.geometry.Vect2F;
import com.blackrook.physics2d.Shape2D;

/**
 * A polygon shape for complex shapes.
 * This must be convex, or collision detection will not work
 * very well. All points are assumed to be around an origin point of (0,0).
 * @author Matthew Tropiano
 */
public class Polygon implements Shape2D
{
	/** This polygon's points, in a clockwise order. */
	protected Point2F[] points;

	/** This polygon's normals, for separating axis tests. */
	protected Vect2F[] normals;
	/** This polygon's center point. */
	protected Point2F centerPoint;
	/** This polygon's radius (to furthest point). */
	protected float squareRadius;
	/** This polygon's half-width. */
	protected float halfWidth;
	/** This polygon's half-height. */
	protected float halfHeight;
	
	/**
	 * Creates a new Polygon.
	 * The points are assumed to be in clockwise order, around the polygon.
	 * @param pts the list of polygon points.
	 */
	public Polygon(Point2F ... pts)
	{
		points = new Point2F[pts.length];
		normals = new Vect2F[pts.length];
		centerPoint = new Point2F();
		squareRadius = 0.0f;
		halfWidth = 0.0f;
		halfHeight = 0.0f;
		float minx = Float.MAX_VALUE;
		float miny = Float.MAX_VALUE;
		float maxx = -Float.MAX_VALUE;
		float maxy = -Float.MAX_VALUE;
		for (int i = 0; i < pts.length; i++)
		{
			points[i] = new Point2F(pts[i]);
			normals[i] = new Vect2F(pts[i], pts[(i+1)%points.length]);
			normals[i].normalize();
			normals[i].leftNormal();
			squareRadius = Math.max(squareRadius, pts[i].squareLength());
			halfWidth = Math.max(halfWidth, Math.abs(pts[i].x));
			halfHeight = Math.max(halfHeight, Math.abs(pts[i].y));
			minx = Math.min(minx, pts[i].x);
			maxx = Math.max(maxx, pts[i].x);
			miny = Math.min(miny, pts[i].y);
			maxy = Math.max(maxy, pts[i].y);
		}
		halfWidth /= 2.0;
		halfHeight /= 2.0;
		centerPoint.set((minx + maxx)/2.0f, (miny + maxy)/2.0f);
	}
	
	@Override
	public float getHalfWidth()
	{
		return halfWidth;
	}

	@Override
	public float getHalfHeight()
	{
		return halfHeight;
	}

	@Override
	public float getRadius()
	{
		return (float)Math.sqrt(squareRadius);
	}

	@Override
	public float getSquareRadius()
	{
		return squareRadius;
	}

	@Override
	public boolean useRadius()
	{
		return false;
	}

	/**
	 * Returns this polygon's center point.
	 */
	public Point2F getCenterPoint()
	{
		return centerPoint;
	}

	/**
	 * Returns this polygon's points.
	 */
	public Point2F[] getPoints()
	{
		return points;
	}

	/**
	 * Returns this polygon's normals (separating axes).
	 */
	public Vect2F[] getNormals()
	{
		return normals;
	}

	@Override
	public String toString()
	{
		return "Polygon(P:"+Arrays.toString(points)+" N:"+Arrays.toString(normals)+
			" Rad:"+Math.sqrt(squareRadius)+" HW:"+halfWidth+", HH:"+halfHeight+"])";
	}

}
