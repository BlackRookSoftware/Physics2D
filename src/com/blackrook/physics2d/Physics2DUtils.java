/*******************************************************************************
 * Copyright (c) 2014 - 2015 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

import com.blackrook.commons.Common;
import com.blackrook.commons.ResettableIterator;
import com.blackrook.commons.hash.Hash;
import com.blackrook.commons.list.List;
import com.blackrook.commons.math.GeometryUtil;
import com.blackrook.commons.math.RMath;
import com.blackrook.commons.math.geometry.Line2D;
import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.commons.math.geometry.Point3D;
import com.blackrook.commons.math.geometry.Vect2D;
import com.blackrook.physics2d.Collision2D.Method;
import com.blackrook.physics2d.shape2d.Box2D;
import com.blackrook.physics2d.shape2d.Circle;
import com.blackrook.physics2d.shape2d.Polygon;

/**
 * Utility library for aiding in collision detection.
 * @author Matthew Tropiano
 */
public final class Physics2DUtils
{
	/** Key for Overlap. */
	private static final String LOCAL_CACHE_NAME = Common.getPackagePathForClass(Physics2DUtils.class) + ".Cache"; 
	
	/**
	 * Cache class for debugging and assisting in calculation of
	 * interval overlap. Not to be used for any end-collision purposes.
	 * This is really just a glorified object pool.
	 */
	private static class Cache
	{
		/** Overlap min A. */
		private final Line2D lineA; 
		/** Overlap min B. */
		private final Line2D lineB; 
		/** Temp Point. */
		private final Point2D point; 
		/** Temp Vector. */
		private final Vect2D vector; 
		/** Temp 3D Point. */
		private final Point3D point3d; 

		/** Separating axis iterator. */
		private ResettableIterator<Vect2D> axisIterator;
		/** Separating axes. */
		private List<Vect2D> axes;
		/** Axis count. */
		private int axisCount;
		
		/** Angle hash  */
		private Hash<Double> angleHash; 
		
		private Cache()
		{
			lineA = new Line2D(0, 0, 0, 0);
			lineB = new Line2D(0, 0, 0, 0);
			point = new Point2D();
			point3d = new Point3D();
			vector = new Vect2D();
			axes = new List<Vect2D>();
			axisCount = 0;
			axisIterator = axes.iterator();
			angleHash = new Hash<Double>();
		}
		
		private void axisReset()
		{
			axisCount = 0;
		}
		
		private void addAxis(double x, double y)
		{
			if (axisCount >= axes.size())
				axes.add(new Vect2D(x, y));
			else
				axes.getByIndex(axisCount).set(x, y);
			axes.getByIndex(axisCount).normalize();
			axisCount++;
		}

		private void addAxisRotated(double x, double y, double degrees)
		{
			if (axisCount >= axes.size())
				axes.add(new Vect2D(x, y));
			else
				axes.getByIndex(axisCount).set(x, y);
			axes.getByIndex(axisCount).normalize();
			if (degrees % 360.0 != 0)
				axes.getByIndex(axisCount).rotateZ(RMath.degToRad(degrees));
			axisCount++;
		}

		private void angleHashReset()
		{
			angleHash.clear();
		}
		
		private void addAngleHash(double angle)
		{
			angleHash.put(angle);
		}

		private boolean angleHashContains(double angle)
		{
			return angleHash.contains(angle);
		}
		
	}

	private Physics2DUtils() {}
	
	/** Grabs thread-specific cache object. */
	private static Cache getCache()
	{
		Cache out = null;
		if ((out = (Cache)Common.getLocal(LOCAL_CACHE_NAME)) == null)
			Common.setLocal(LOCAL_CACHE_NAME, out = new Cache());
		return out;
	}

	/**
	 * Tests a collision between two bodies.
	 * Sets the source and the target on the collision object.
	 * @param collision the collision data object.
	 * @param model the physics model to use for object attributes.
	 * @param bodyA the first body.
	 * @param bodyB the second body.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static <T> boolean testCollision(Collision2D<T> collision, Physics2DModel<T> model, T bodyA, T bodyB)
	{
		long time = System.nanoTime();
		boolean collide = false;
		
		collision.source = bodyA;
		collision.target = bodyB;
		collision.axisCount = 0;
		collision.incidentPoint.set(0, 0);
		collision.incidentVector.set(0, 0);
		
		Shape2D shapeA = model.getCollisionShape(bodyA);
		Shape2D shapeB = model.getCollisionShape(bodyB);
		
		if (shapeA instanceof Circle)
		{
			if (shapeB instanceof Circle)
				collide = testStationaryCollision(collision, model, (Circle)shapeA, (Circle)shapeB);
			else if (shapeB instanceof Box2D)
				collide = testStationaryCollision(collision, model, (Circle)shapeA, (Box2D)shapeB);
			else
				collide = testSeparatingAxisCollision(model, collision, bodyA, bodyB);
		}
		else if (shapeA instanceof Box2D)
		{
			if (shapeB instanceof Box2D)
				collide = testStationaryCollision(collision, model, (Box2D)shapeA, (Box2D)shapeB);
			else if (shapeB instanceof Circle)
				collide = testStationaryCollision(collision, model, (Box2D)shapeA, (Circle)shapeB);
			else
				collide = testSeparatingAxisCollision(model, collision, bodyA, bodyB);
		}
		else
		{
			collide = testSeparatingAxisCollision(model, collision, bodyA, bodyB);
		}
		
		collision.calcNanos = System.nanoTime() - time;
		
		return collide;
	}
	
	/**
	 * Tests if two shapes collide. Circle vs. Circle.
	 * Sets the source and the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param model the collision model to use.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static <T> boolean testStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Circle source, Circle target)
	{
		collision.method = Method.CIRCLE_TO_CIRCLE;
		
		Cache cache = getCache();

		model.getCollisionCenter(collision.source, cache.point);
		double spx = cache.point.x;
		double spy = cache.point.y;
		
		model.getCollisionCenter(collision.target, cache.point);
		double tpx = cache.point.x;
		double tpy = cache.point.y;

		double sourceRadius = source.getRadius();
		double targetRadius = target.getRadius();
		if (RMath.getIntersectionCircle(spx, spy, sourceRadius, tpx, tpy, targetRadius))
		{
			RMath.getOverlapCircle(collision.incidentVector, collision.incidentPoint, spx, spy, sourceRadius, tpx, tpy, targetRadius);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests if two shapes collide. Circle vs. Box.
	 * Sets the source and the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param model the collision model to use.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static <T> boolean testStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Circle source, Box2D target)
	{
		collision.method = Method.CIRCLE_TO_BOX;
	
		Cache cache = getCache();

		model.getCollisionCenter(collision.source, cache.point);
		double spx = cache.point.x;
		double spy = cache.point.y;
		
		model.getCollisionCenter(collision.target, cache.point);
		double bcx = cache.point.x;
		double bcy = cache.point.y;

		model.getCollisionHalfWidths(collision.target, cache.point);
		double bhw = cache.point.x; 
		double bhh = cache.point.y; 

		double srcRadius = source.getRadius();
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		if (RMath.getIntersectionCircleBox(spx, spy, srcRadius, bcx, bcy, bhw, bhh))
		{
			RMath.getOverlapCircleBox(incVect, incPoint, spx, spy, srcRadius, bcx, bcy, bhw, bhh);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests if two shapes collide. Box vs. Circle.
	 * Sets the source and the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static <T> boolean testStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Box2D source, Circle target)
	{
		collision.method = Method.BOX_TO_CIRCLE;
	
		Cache cache = getCache();

		model.getCollisionCenter(collision.target, cache.point);
		double cpx = cache.point.x;
		double cpy = cache.point.y;
		
		model.getCollisionCenter(collision.source, cache.point);
		double bcx = cache.point.x;
		double bcy = cache.point.y;

		model.getCollisionHalfWidths(collision.source, cache.point);
		double bhw = cache.point.x; 
		double bhh = cache.point.y; 

		double targRadius = target.getRadius();
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
	
		if (RMath.getIntersectionCircleBox(cpx, cpy, targRadius, bcx, bcy, bhw, bhh))
		{
			RMath.getOverlapCircleBox(incVect, incPoint, cpx, cpy, targRadius, bcx, bcy, bhw, bhh);
			incVect.negate();
			return true;
		}
		
		return false;
	}

	/**
	 * Tests if two shapes collide. AABB vs. AABB.
	 * Sets the source and the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static <T> boolean testStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Box2D source, Box2D target)
	{
		collision.method = Method.BOX_TO_BOX;
	
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		Cache cache = getCache();
		
		model.getCollisionCenter(collision.source, cache.point);
		double spx = cache.point.x;
		double spy = cache.point.y;

		model.getCollisionHalfWidths(collision.source, cache.point);
		double shw = cache.point.x;
		double shh = cache.point.y;

		model.getCollisionCenter(collision.target, cache.point);
		double tpx = cache.point.x;
		double tpy = cache.point.y;
		
		model.getCollisionHalfWidths(collision.target, cache.point);
		double thw = cache.point.x;
		double thh = cache.point.y;

		if (RMath.getIntersectionBox(spx, spy, shw, shh, tpx, tpy, thw, thh))
		{
			RMath.getOverlapBox(incVect, incPoint, spx, spy, shw, shh, tpx, tpy, thw, thh);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests a raycasting collision on a body.
	 * The direction of the ray influences the incident point.
	 * Sets only the target on the collision object.
	 * @param collision the collision data object.
	 * @param model the physics model to use for object attributes.
	 * @param line the line for intersection.
	 * @param body the test body.
	 * @return true if a collision occurs shapes collide, false otherwise.
	 */
	public static <T> boolean testRaycastCollision(Collision2D<T> collision, Physics2DModel<T> model, Line2D line, T body)
	{
		collision.method = Method.SEPARATING_AXIS;
		
		long time = System.nanoTime();
		boolean collide = false;
		
		collision.source = null;
		collision.target = body;
		collision.axisCount = 0;
		collision.incidentPoint.set(0, 0);
		collision.incidentVector.set(0, 0);
		
		Shape2D shape = model.getCollisionShape(body);
		
		if (shape instanceof Circle)
			collide = testRaycastStationaryCollision(collision, model, line, (Circle)shape);
		else if (shape instanceof Box2D)
			collide = testRaycastStationaryCollision(collision, model, line, (Box2D)shape);
		else if (shape instanceof Polygon)
			collide = testRaycastStationaryCollision(collision, model, line, (Polygon)shape);
		else
			collide = testRaycastSeparatingAxisCollision(model, collision, line, body);

		collision.calcNanos = System.nanoTime() - time;
		
		return collide;
	}

	/**
	 * Tests a raycasting collision on a stationary circle.
	 * The direction of the ray influences the incident point.
	 * Sets only the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param model the physics model to use for object attributes.
	 * @param line the line for intersection.
	 * @param body the test body.
	 * @return true if a collision occurs shapes collide, false otherwise.
	 */
	public static <T> boolean testRaycastStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Line2D line, Circle body)
	{
		collision.method = Method.LINE_TO_CIRCLE;
		
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		Cache cache = getCache();
				
		model.getCollisionCenter(collision.target, cache.point);
		double cpx = cache.point.x;
		double cpy = cache.point.y;
		
		double radius = body.getRadius();
		
		if (RMath.getIntersectionLineCircle(line.pointA.x, line.pointA.y, line.pointB.x, line.pointB.y, cpx, cpy, radius))
		{
			RMath.getOverlapLineCircle(incVect, incPoint, line.pointA.x, line.pointA.y, line.pointB.x, line.pointB.y, cpx, cpy, radius);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests a raycasting collision on a stationary bounding box.
	 * The direction of the ray influences the incident point.
	 * Sets only the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param model the physics model to use for object attributes.
	 * @param line the line for intersection.
	 * @param body the test body.
	 * @return true if a collision occurs shapes collide, false otherwise.
	 */
	public static <T> boolean testRaycastStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Line2D line, Box2D body)
	{
		collision.method = Method.LINE_TO_BOX;
		
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		Cache cache = getCache();
		model.getCollisionCenter(collision.target, cache.point);
		double cpx = cache.point.x;
		double cpy = cache.point.y;

		double chw = body.getHalfWidth();
		double chh = body.getHalfWidth();
		
		if (RMath.getIntersectionLineBox(line.pointA.x, line.pointA.y, line.pointB.x, line.pointB.y, cpx, cpy, chw, chh))
		{
			RMath.getOverlapLineBox(incVect, incPoint, line.pointA.x, line.pointA.y, line.pointB.x, line.pointB.y, cpx, cpy, chw, chh);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests a raycasting collision on a stationary polygon.
	 * The direction of the ray influences the incident point.
	 * Sets only the target on the collision object.
	 * @param collision the collision data object. source and target should already be set.
	 * @param model the physics model to use for object attributes.
	 * @param line the line for intersection.
	 * @param body the test body.
	 * @return true if a collision occurs shapes collide, false otherwise.
	 */
	public static <T> boolean testRaycastStationaryCollision(Collision2D<T> collision, Physics2DModel<T> model, Line2D line, Polygon body)
	{
		collision.method = Method.LINE_TO_POLYGON;
		
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		Cache cache = getCache();
		model.getCollisionCenter(collision.target, cache.point);
		double cpx = cache.point.x;
		double cpy = cache.point.y;

		// intersection incident points.
		double ipx1 = 0.0;
		double ipy1 = 0.0;
		double ipx2 = 0.0;
		double ipy2 = 0.0;

		// first intersection normals.
		double ipx1n = 0.0;
		double ipy1n = 0.0;
		
		// A line crossing through a convex shape can - at most - bisect two sides.
		// These points are the incident, but the closest to the line start is the incident point recorded.
		boolean firstCollision = false;
		boolean secondCollision = false;
		
		Point2D[] points = body.getPoints();
		
		for (int i = 0; i < points.length && !(firstCollision && secondCollision); i++)
		{
			double sx, sy, tx, ty;
			
			// set line points for intersection test.
			sx = cpx + points[i].x;
			sy = cpy + points[i].y;
			int p = (i + 1) % points.length;
			tx = cpx + points[p].x;
			ty = cpy + points[p].y;
			
			boolean intersect = test2DSegments(cache.point, line.pointA.x, line.pointA.y, line.pointB.x, line.pointB.y, sx, sy, tx, ty);
			if (intersect)
			{
				if (!firstCollision)
				{
					firstCollision = true;
					ipx1 = cache.point.x;
					ipy1 = cache.point.y;
					ipx1n = - (points[p].y - points[i].y); 
					ipy1n = points[p].x - points[i].x; 
				}
				else
				{
					secondCollision = true;
					ipx2 = cache.point.x;
					ipy2 = cache.point.y;
				}
			}
		}
		
		if (firstCollision)
		{
			if (secondCollision)
			{
				testRaycastGetProjectedIncident(model, collision, line, incVect, incPoint, ipx1, ipy1, false, ipx2, ipy2);
			}
			else
			{
				// use "dot product" incident.
				double dotp = RMath.getVectorUnitDotProduct(line.pointB.x - line.pointA.x, line.pointB.y - line.pointA.y, line.pointB.x - cpx, line.pointB.y - cpy);

				if (dotp < 0.0)
				{
					// this incident vector is incorrect.
					incPoint.x = ipx1;
					incPoint.y = ipy1;
					
					cache.vector.set(ipx1n, ipy1n);
					cache.lineA.pointA.set(line.pointB);
					cache.lineA.pointB.set(ipx1, ipy1);
					cache.lineA.pointA.projectOnto(cache.vector);
					cache.lineA.pointB.projectOnto(cache.vector);
					
					incVect.set(cache.lineA.pointA, cache.lineA.pointB);
				}
				else
				{
					testRaycastGetProjectedIncident(model, collision, line, incVect, incPoint, ipx1, ipy1, true, ipx2, ipy2);
				}
				
			}
			return true;
		}
		else
			return false;
	}

	/**
	 * Axis overlap test.
	 * Smallest overlap is written to the smallestOverlap vector.
	 * @param bodyA the first body.
	 * @param bodyB the second body.
	 * @param axis the separating axis normal.
	 * @param overlapOut the overlap vector.
	 * @return true if overlap, false if not.
	 */
	public static <T> boolean axisOverlapTest(Physics2DModel<T> model, T bodyA, T bodyB, Vect2D axis, Vect2D overlapOut)
	{
		Cache cache = getCache();
		
		projectShape2D(model, bodyA, axis, cache.lineA);
		projectShape2D(model, bodyB, axis, cache.lineB);
	
		boolean overlap = GeometryUtil.lineOverlaps(
			cache.lineA.pointA, cache.lineA.pointB, 
			cache.lineB.pointA, cache.lineB.pointB,	overlapOut
		); 
		return overlap;
	}

	/**
	 * Projects a body onto a vector and the line is saved into a line.
	 * @param body2d the body to project.
	 * @param axis the vector to project the shape onto.
	 * @param out the output line. 
	 */
	public static <T> void projectShape2D(Physics2DModel<T> model, T body2d, Vect2D axis, Line2D out)
	{
		Shape2D shape = model.getCollisionShape(body2d);
		
		if (shape instanceof Box2D)
			projectBox(model, (Box2D)shape, body2d, axis, out);
		else if (shape instanceof Circle)
			projectCircle(model, (Circle)shape, body2d, axis, out);
		else if (shape instanceof Polygon)
			projectPolygon(model, (Polygon)shape, body2d, axis, out);
	}

	/** Project Circle */
	public static <T> void projectCircle(Physics2DModel<T> model, Circle circle, T body2d, Vect2D axis, Line2D out)
	{
		Cache cache = getCache();
		
		model.getCollisionCenter(body2d, cache.point);

		double ox = cache.point.x;
		double oy = cache.point.y;
		
		RMath.getProjectedCircle(out, axis.x, axis.y, ox, oy, circle.getRadius());
	}

	/** Project AABB */
	public static <T> void projectBox(Physics2DModel<T> model, Box2D aabb, T body2d, Vect2D axis, Line2D out)
	{
		Cache cache = getCache();
		
		model.getCollisionCenter(body2d, cache.point);

		double ox = cache.point.x;
		double oy = cache.point.y;
		
		double hw = aabb.getHalfWidth();
		double hh = aabb.getHalfHeight();
	
		RMath.getProjectedBox(out, axis.x, axis.y, ox, oy, hw, hh);
	}

	/** Project Polygon */
	public static <T> void projectPolygon(Physics2DModel<T> model, Polygon polygon, T body2d, Vect2D axis, Line2D out)
	{
		double minx = Double.MAX_VALUE;
		double miny = Double.MAX_VALUE;
		double maxx = -Double.MAX_VALUE;
		double maxy = -Double.MAX_VALUE;

		Cache cache = getCache();
	
		model.getCollisionCenter(body2d, cache.point); 
		
		double bcx = cache.point.x;
		double bcy = cache.point.y;
		
		for (Point2D p : polygon.getPoints())
		{
			cache.point.set(p.x, p.y);
			
			model.getCollisionRotation(body2d, cache.point3d);
			double rotationZ = cache.point3d.z;
			if (rotationZ != 0.0)
				cache.point.rotateZ(RMath.degToRad(rotationZ));
			
			cache.point.set(cache.point.x + bcx, cache.point.y + bcy);
			cache.point.projectOnto(axis);
			minx = Math.min(cache.point.x, minx);
			miny = Math.min(cache.point.y, miny);
			maxx = Math.max(cache.point.x, maxx);
			maxy = Math.max(cache.point.y, maxy);
		}
		
		boolean swap = axis.x < 0 ^ axis.y < 0;
		
		if (swap)
		{
			out.pointA.set(maxx, miny);
			out.pointB.set(minx, maxy);
		}
		else
		{
			out.pointA.set(minx, miny);
			out.pointB.set(maxx, maxy);
		}
		
		out.pointA.projectOnto(axis);
		out.pointB.projectOnto(axis);
	}

	/**
	 * Returns (val - min) if val is closer to min than max, (max - val) otherwise.
	 * Result is always positive. 
	 */
	public static double closerComponent(double val, double min, double max)
	{
		return Math.abs(val - min) < Math.abs(val - max) ? (val - min) : (max - val);
	}

	/** Caches the separating axes for a set of collision bodies. */
	private static <T> void cacheSeparatingAxes(Physics2DModel<T> model, T bodyA, T bodyB)
	{
		Shape2D shapeA = model.getCollisionShape(bodyA);
		Shape2D shapeB = model.getCollisionShape(bodyB);
		
		Cache cache = getCache();
		cache.axisReset();
		
		if (shapeA instanceof Circle)
			cacheSeparatingAxesCircle(model, bodyA, bodyB);
		else
		{
			if (shapeA instanceof Box2D)
				cacheSeparatingAxesBox(model, bodyA);
			else if (shapeA instanceof Polygon)
				cacheSeparatingAxesPolygon(model, bodyA);
	
			if (shapeB instanceof Circle)
				cacheSeparatingAxesCircle(model, bodyB, bodyA);
			else if (shapeB instanceof Box2D)
				cacheSeparatingAxesBox(model, bodyB);
			else if (shapeB instanceof Polygon)
				cacheSeparatingAxesPolygon(model, bodyB);
		}
	}

	/** Caches the separating axes for a circle. */
	private static <T> void cacheSeparatingAxesCircle(Physics2DModel<T> model, T body, T otherBody)
	{
		Cache cache = getCache();
		
		model.getCollisionCenter(body, cache.point); 
		double bcx = cache.point.x;
		double bcy = cache.point.y;
		model.getCollisionCenter(otherBody, cache.point); 
		double ocx = cache.point.x;
		double ocy = cache.point.y;
		
		// center to other center.
		cache.addAxis(ocx - bcx, ocy - bcy);
		
	}
	
	/** Caches the separating axes for an AABB. */
	private static <T> void cacheSeparatingAxesBox(Physics2DModel<T> model, T body)
	{
		Cache cache = getCache();
		
		// add actual axes.
		cache.addAxis(1.0, 0.0);
		cache.addAxis(0.0, 1.0);
		
	}

	/** Caches the separating axes for an Polygon. */
	private static <T> void cacheSeparatingAxesPolygon(Physics2DModel<T> model, T body)
	{
		Cache cache = getCache();
		
		// add polygonal normals.
		Polygon polygon = (Polygon)model.getCollisionShape(body);
		for (Vect2D normal : polygon.getNormals())
		{
			model.getCollisionRotation(body, cache.point3d);
			double rotationZ = cache.point3d.z;
			cache.addAxisRotated(normal.x, normal.y, rotationZ);
		}
		
	}

	/**
	 * 
	 * @param model the physics model to use for object attributes.
	 * @param collision the collision data object. target should already be set.
	 * @param line the line for intersection.
	 * @param incVect output incident vector.
	 * @param incPoint output incident point.
	 * @param ipx1 first incident point, x-component.
	 * @param ipy1 first incident point, y-component.
	 * @param ignore2 if true, ignore 2nd incident.
	 * @param ipx2 second incident point, x-component.
	 * @param ipy2 second incident point, y-component.
	 */
	private static <T> void testRaycastGetProjectedIncident(Physics2DModel<T> model, Collision2D<T> collision, Line2D line, Vect2D incVect, Point2D incPoint, double ipx1, double ipy1, boolean ignore2, double ipx2, double ipy2)
	{
		if (!ignore2)
		{
			// what is closer to the line start?
			if (RMath.getLineLengthSquared(ipx1, ipy1, line.pointA.x, line.pointA.y) < RMath.getLineLengthSquared(ipx2, ipy2, line.pointA.x, line.pointA.y))
			{
				incPoint.x = ipx1;
				incPoint.y = ipy1;
			}
			else
			{
				incPoint.x = ipx2;
				incPoint.y = ipy2;
			}
		}
		else
		{
			incPoint.x = ipx1;
			incPoint.y = ipy1;
		}
	
		Cache cache = getCache();
		cache.vector.set(line.pointA, line.pointB);
		cache.vector.leftNormal();
		projectShape2D(model, collision.target, cache.vector, cache.lineA);
		
		if (RMath.getVectorLengthSquared(cache.lineA.pointA.x, cache.lineA.pointA.y) < RMath.getVectorLengthSquared(cache.lineA.pointB.x, cache.lineA.pointB.y))
			incVect.set(cache.lineA.pointA.x, cache.lineA.pointA.y);
		else
			incVect.set(cache.lineA.pointB.x, cache.lineA.pointB.y);
	}

	/**
	 * Tests collisions using separating axis.
	 * @param bodyA the first body. 
	 * @param bodyB the second body.
	 * @param collision the collision information.
	 * TODO: Incident points for separating axis test.
	 */
	private static <T> boolean testRaycastSeparatingAxisCollision(Physics2DModel<T> model, Collision2D<T> collision, Line2D line, T bodyB)
	{
		// TODO: Finish.
		return false;
	}
	
	/**
	 * Tests collisions using separating axis.
	 * @param bodyA the first body. 
	 * @param bodyB the second body.
	 * @param collision the collision information.
	 * TODO: Incident points for separating axis test.
	 */
	private static <T> boolean testSeparatingAxisCollision(Physics2DModel<T> model, Collision2D<T> collision, T bodyA, T bodyB)
	{
		collision.method = Method.SEPARATING_AXIS;
	
		Cache cache = getCache();
		cache.angleHashReset();
		
		boolean stillGood = true;
	
		// get separating axes.
		cacheSeparatingAxes(model, bodyA, bodyB);
		ResettableIterator<Vect2D> rit = cache.axisIterator;
		rit.reset();
		while (stillGood && rit.hasNext())
		{
			Vect2D axis = rit.next();
			double angle = (double)RMath.getVectorAngleDegrees(axis.x, axis.y);
			angle = angle > 180.0 ? angle - 180.0 : angle;
	
			// if I used this axis already, else test. 
			if (!cache.angleHashContains(angle))
			{
				cache.addAngleHash(angle);
				collision.axisCount++;
				stillGood = axisOverlapTest(model, bodyA, bodyB, axis, cache.vector);
				if (stillGood)
				{
					double ovllen = cache.vector.length();
					if ((ovllen > 0.0 && ovllen < collision.incidentVector.length()) || collision.incidentVector.isZero())
						collision.incidentVector.set(cache.vector);
				}
			}
		}
		
		if (stillGood)
			collision.incidentVector.negate();
		
		return stillGood;
	}

	/** Test if two lines intersect and sets the incident point p. */
	private static boolean test2DSegments(Point2D p, double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
	{
		double t = RMath.getIntersectionLine(ax, ay, bx, by, cx, cy, dx, dy);
		if (!Double.isNaN(t))
		{
			RMath.getOverlapPoint(p, ax, ay, bx, by, t);
			return true;
		}
		else
			return false;
	}
	
}
