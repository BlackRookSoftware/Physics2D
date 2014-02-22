/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
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
import com.blackrook.commons.math.geometry.Point2F;
import com.blackrook.commons.math.geometry.Vect2D;
import com.blackrook.commons.math.geometry.Vect2F;
import com.blackrook.physics2d.Collision2D.Method;
import com.blackrook.physics2d.shape2d.AABB;
import com.blackrook.physics2d.shape2d.Circle;
import com.blackrook.physics2d.shape2d.Polygon;

/**
 * Utility library for aiding in collision detection.
 * @author Matthew Tropiano
 */
public final class PhysicsUtils2D
{
	/** Key for Overlap. */
	private static final String LOCAL_CACHE_NAME = Common.getPackagePathForClass(PhysicsUtils2D.class) + ".Cache"; 
	
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

		/** Point iterator. */
		private ResettableIterator<Point2D> pointIterator;
		/** Points. */
		private List<Point2D> points;
		/** Point count. */
		private int pointCount;
		
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
			vector = new Vect2D();
			axes = new List<Vect2D>();
			axisCount = 0;
			axisIterator = axes.iterator();
			points = new List<Point2D>();
			pointCount = 0;
			pointIterator = points.iterator();
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

		private void pointReset()
		{
			pointCount = 0;
		}
		
		private void addPoint(double x, double y)
		{
			if (pointCount >= points.size())
				points.add(new Point2D(x, y));
			else
				points.getByIndex(pointCount).set(x, y);
			pointCount++;
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

	private PhysicsUtils2D() {}
	
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
	 * @param bodyA the first body.
	 * @param bodyB the second body.
	 * @param collision the collision data object. source and target should already be set.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static boolean testCollision(CollisionBody2D bodyA, CollisionBody2D bodyB, Collision2D<?> collision)
	{
		long time = System.nanoTime();
		boolean collide = false;
		
		collision.axisCount = 0;

		collision.incidentVector.set(0, 0);
		
		// use better models for known shapes.
		if (!checkSweep(bodyA) && !checkSweep(bodyB))
		{
			Shape2D shapeA = bodyA.getCollisionShape();
			Shape2D shapeB = bodyB.getCollisionShape();
			
			if (shapeA instanceof Circle)
			{
				if (shapeB instanceof Circle)
					collide = testStationaryCollision((Circle)shapeA, (Circle)shapeB, collision);
				else if (shapeB instanceof AABB)
					collide = testStationaryCollision((Circle)shapeA, (AABB)shapeB, collision);
				else
					collide = testSeparatingAxisCollision(bodyA, bodyB, collision);
			}
			else if (shapeA instanceof AABB)
			{
				if (shapeB instanceof AABB)
					collide = testStationaryCollision((AABB)shapeA, (AABB)shapeB, collision);
				else if (shapeB instanceof Circle)
					collide = testStationaryCollision((AABB)shapeA, (Circle)shapeB, collision);
				else
					collide = testSeparatingAxisCollision(bodyA, bodyB, collision);
			}
			else
				collide = testSeparatingAxisCollision(bodyA, bodyB, collision);
		}

		// use other model.
		else
			collide = testSeparatingAxisCollision(bodyA, bodyB, collision);
		
		collision.calcNanos = System.nanoTime() - time;
		
		return collide;
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
	public static boolean axisOverlapTest(CollisionBody2D bodyA, CollisionBody2D bodyB, Vect2D axis, Vect2D overlapOut)
	{
		Cache cache = getCache();
		
		projectShape2D(bodyA, axis, cache.lineA);
		projectShape2D(bodyB, axis, cache.lineB);

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
	public static void projectShape2D(CollisionBody2D body2d, Vect2D axis, Line2D out)
	{
		Shape2D shape = body2d.getCollisionShape();
		
		if (shape instanceof AABB)
			projectAABB((AABB)shape, body2d, axis, out);
		else if (shape instanceof Circle)
			projectCircle((Circle)shape, body2d, axis, out);
		else if (shape instanceof Polygon)
			projectPolygon((Polygon)shape, body2d, axis, out);
	}
	
	/** Project Circle */
	public static void projectCircle(Circle circle, CollisionBody2D body2d, Vect2D axis, Line2D out)
	{
		double ox = body2d.getObjectCenterX();
		double oy = body2d.getObjectCenterY();
		
		double rad = circle.getRadius();
		
		boolean swap = axis.x < 0 ^ axis.y < 0;
		
		if (axis.x != 0.0)
		{
			double theta = Math.atan(axis.y / axis.x);
			if (theta < 0) theta = -theta;
			double dx = rad * Math.cos(theta);
			double dy = rad * Math.sin(theta);

			if (swap)
			{
				out.pointA.x = ox - dx;
				out.pointA.y = oy + dy;
				out.pointB.x = ox + dx;
				out.pointB.y = oy - dy;
			}
			else
			{
				out.pointA.x = ox - dx;
				out.pointA.y = oy - dy;
				out.pointB.x = ox + dx;
				out.pointB.y = oy + dy;
			}
		}
		else
		{
			out.pointA.x = ox;
			out.pointA.y = oy - rad;
			out.pointB.x = ox;
			out.pointB.y = oy + rad;
		}
		
		if (checkSweep(body2d)) sweepAdjust(body2d, axis, out);
		
		out.pointA.projectOnto(axis);
		out.pointB.projectOnto(axis);
	}

	/** Project AABB */
	public static void projectAABB(AABB aabb, CollisionBody2D body2d, Vect2D axis, Line2D out)
	{
		double ox = body2d.getObjectCenterX();
		double oy = body2d.getObjectCenterY();
		
		double hw = aabb.getHalfWidth();
		double hh = aabb.getHalfHeight();

		boolean swap = axis.x < 0 ^ axis.y < 0;
		
		if (axis.x != 0.0)
		{
			double theta = Math.atan(axis.y / axis.x);
			if (theta < 0) theta = -theta;
			double hl = (hh * Math.sin(theta)) + (hw * Math.cos(theta));
			double dx = hl * Math.cos(theta);
			double dy = hl * Math.sin(theta);
			
			if (swap)
			{
				out.pointA.x = ox - dx;
				out.pointA.y = oy + dy;
				out.pointB.x = ox + dx;
				out.pointB.y = oy - dy;
			}
			else
			{
				out.pointA.x = ox - dx;
				out.pointA.y = oy - dy;
				out.pointB.x = ox + dx;
				out.pointB.y = oy + dy;
			}
		}
		else
		{
			out.pointA.x = ox;
			out.pointA.y = oy - hh;
			out.pointB.x = ox;
			out.pointB.y = oy + hh;
		}

		if (checkSweep(body2d)) sweepAdjust(body2d, axis, out);
		
		out.pointA.projectOnto(axis);
		out.pointB.projectOnto(axis);
	}

	/** Project Polygon */
	public static void projectPolygon(Polygon polygon, CollisionBody2D body2d, Vect2D axis, Line2D out)
	{
		double minx = Double.MAX_VALUE;
		double miny = Double.MAX_VALUE;
		double maxx = -Double.MAX_VALUE;
		double maxy = -Double.MAX_VALUE;

		Cache cache = getCache();
		for (Point2F p : polygon.getPoints())
		{
			cache.point.set(p.x, p.y);
			if (body2d.getCollisionRotationZ() != 0.0)
				cache.point.rotateZ(RMath.degToRad(body2d.getCollisionRotationZ()));
			cache.point.set(cache.point.x + body2d.getObjectCenterX(), cache.point.y + body2d.getObjectCenterY());
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
		
		if (checkSweep(body2d)) sweepAdjust(body2d, axis, out);
		
		out.pointA.projectOnto(axis);
		out.pointB.projectOnto(axis);
	}
	
	/** Checks if this object is swept or not. */
	private static boolean checkSweep(CollisionBody2D body2d)
	{
		return body2d.getCollisionVelocityX() != 0.0 || body2d.getCollisionVelocityY() != 0.0;
	}

	/** Adjust for sweep (velocity). */
	public static void sweepAdjust(CollisionBody2D body2d, Vect2D axis, Line2D out)
	{
		Cache cache = getCache();
		cache.vector.set(body2d.getCollisionVelocityX(), body2d.getCollisionVelocityY());
		cache.vector.projectOnto(axis);

		boolean swap = axis.x < 0 ^ axis.y < 0;
		
		cache.pointReset();
		cache.addPoint(out.pointA.x, out.pointA.y);
		cache.addPoint(out.pointB.x, out.pointB.y);
		cache.addPoint(out.pointA.x - cache.vector.x, out.pointA.y - cache.vector.y);
		cache.addPoint(out.pointB.x - cache.vector.x, out.pointB.y - cache.vector.y);
		
		double minx = Double.MAX_VALUE;
		double miny = Double.MAX_VALUE;
		double maxx = -Double.MAX_VALUE;
		double maxy = -Double.MAX_VALUE;

		ResettableIterator<Point2D> rit = cache.pointIterator;
		rit.reset();
		while (rit.hasNext())
		{
			Point2D p = rit.next();
			minx = Math.min(p.x, minx);
			miny = Math.min(p.y, miny);
			maxx = Math.max(p.x, maxx);
			maxy = Math.max(p.y, maxy);
		}

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
	}
	
	/** Caches the separating axes for a set of collision bodies. */
	public static void cacheSeparatingAxes(CollisionBody2D bodyA, CollisionBody2D bodyB)
	{
		Shape2D shapeA = bodyA.getCollisionShape();
		Shape2D shapeB = bodyB.getCollisionShape();
		
		Cache cache = getCache();
		cache.axisReset();
		
		if (shapeA instanceof Circle)
			cacheSeparatingAxesCircle(bodyA, bodyB);
		else
		{
			if (shapeA instanceof AABB)
				cacheSeparatingAxesAABB(bodyA);
			else if (shapeA instanceof Polygon)
				cacheSeparatingAxesPolygon(bodyA);

			if (shapeB instanceof Circle)
				cacheSeparatingAxesCircle(bodyB, bodyA);
			else if (shapeB instanceof AABB)
				cacheSeparatingAxesAABB(bodyB);
			else if (shapeB instanceof Polygon)
				cacheSeparatingAxesPolygon(bodyB);
		}

	}
	
	/** Caches the separating axes for a circle. */
	private static void cacheSeparatingAxesCircle(CollisionBody2D body, CollisionBody2D otherBody)
	{
		Cache cache = getCache();
		
		// center to other center.
		cache.addAxis(
			otherBody.getObjectCenterX() - body.getObjectCenterX(), 
			otherBody.getObjectCenterY() - body.getObjectCenterY()
			);
		
		// add axes for movement.
		if (checkSweep(body))
		{
			// velocity normals
			cache.addAxis(-body.getCollisionVelocityY(), body.getCollisionVelocityX());
			// previous position center to other center.
			cache.addAxis(
				otherBody.getObjectCenterX() - (body.getObjectCenterX() - body.getCollisionVelocityX()), 
				otherBody.getObjectCenterY() - (body.getObjectCenterY() - body.getCollisionVelocityY())
				);
		}
		
	}
	
	/** Caches the separating axes for an AABB. */
	private static void cacheSeparatingAxesAABB(CollisionBody2D body)
	{
		Cache cache = getCache();
		
		// add actual axes.
		cache.addAxis(1.0, 0.0);
		cache.addAxis(0.0, 1.0);
		
		// add axes for movement.
		if (checkSweep(body))
		{
			// velocity normals
			cache.addAxis(-body.getCollisionVelocityY(), body.getCollisionVelocityX());
		}
		
	}

	/** Caches the separating axes for an Polygon. */
	private static void cacheSeparatingAxesPolygon(CollisionBody2D body)
	{
		Cache cache = getCache();
		
		// add polygonal normals.
		Polygon polygon = (Polygon)body.getCollisionShape();
		for (Vect2F normal : polygon.getNormals())
			cache.addAxisRotated(normal.x, normal.y, body.getCollisionRotationZ());
		
		// add axes for movement.
		if (checkSweep(body))
		{
			// velocity normals
			cache.addAxis(-body.getCollisionVelocityY(), body.getCollisionVelocityX());
		}
		
	}

	/**
	 * Tests if two shapes collide. Circle vs. Circle.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @param collision the collision data object. source and target should already be set.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static boolean testStationaryCollision(Circle source, Circle target, Collision2D<?> collision)
	{
		collision.method = Method.CIRCLE_TO_CIRCLE;
		
		double spx = collision.source.getObjectCenterX();
		double spy = collision.source.getObjectCenterY();
		double tpx = collision.target.getObjectCenterX();
		double tpy = collision.target.getObjectCenterY();
		
		double cdist = distance(spx, spy, tpx, tpy);
		double rdist = source.getRadius() + target.getRadius();
		if (cdist < rdist)
		{
			Vect2D inc = collision.incidentVector;
			double dx = 0;
			double dy = 0;
			inc.set(tpx - spx, tpy - spy);
			inc.setLength(source.getRadius());
			dx = inc.x;
			dy = inc.y;
			inc.setLength(rdist - cdist);
			collision.incidentPoint.set(spx + dx - inc.x, spy + dy - inc.y);
			return true;
		}
		
		return false;
	}

	/**
	 * Tests if two shapes collide. Circle vs. Box.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @param collision the collision data object. source and target should already be set.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static boolean testStationaryCollision(Circle source, AABB target, Collision2D<?> collision)
	{
		collision.method = Method.CIRCLE_TO_BOX;

		double spx = collision.source.getObjectCenterX();
		double spy = collision.source.getObjectCenterY();
	
		double tx0 = collision.target.getObjectCenterX() - target.getHalfWidth();
		double tx1 = collision.target.getObjectCenterX() + target.getHalfWidth();
		double ty0 = collision.target.getObjectCenterY() - target.getHalfHeight();
		double ty1 = collision.target.getObjectCenterY() + target.getHalfHeight();
	
		double srcRadius = source.getRadius();
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		// Voronoi Region Test.
		if (spx < tx0)
		{
			if (spy < ty0)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx0, ty0);
			else if (spy > ty1)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx0, ty1);
			else
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx0, spy);
		}
		else if (spx > tx1)
		{
			if (spy < ty0)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx1, ty0);
			else if (spy > ty1)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx1, ty1);
			else
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, tx1, spy);
		}
		else
		{
			if (spy < ty0)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, spx, ty0);
			else if (spy > ty1)
				return testCircleAABBIncidents(incVect, incPoint, srcRadius, spx, spy, spx, ty1);
			else // circle center is inside box
			{
				double closeX = closerComponent(spx, tx0, tx1);
				double closeY = closerComponent(spy, ty0, ty1);
				if (closeX < closeY)
				{
					double tpx = collision.target.getObjectCenterX();
					if (spx < tpx)
					{
						incVect.set(closeX + srcRadius, 0);
						incPoint.set(tx0, spy);
					}
					else
					{
						incVect.set(-closeX - srcRadius, 0);
						incPoint.set(tx1, spy);
					}
				}
				else
				{
					double tpy = collision.target.getObjectCenterY();
					if (spy < tpy)
					{
						incVect.set(0, closeY + srcRadius);
						incPoint.set(spx, ty0);
					}
					else
					{
						incVect.set(0, -closeY - srcRadius);
						incPoint.set(spx, ty1);
					}
				}
				return true;
			}
		}
	}
	
	/**
	 * Tests if two shapes collide. Box vs. Circle.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @param collision the collision data object. source and target should already be set.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static boolean testStationaryCollision(AABB source, Circle target, Collision2D<?> collision)
	{
		collision.method = Method.BOX_TO_CIRCLE;

		double cpx = collision.target.getObjectCenterX();
		double cpy = collision.target.getObjectCenterY();

		double bx0 = collision.source.getObjectCenterX() - target.getHalfWidth();
		double bx1 = collision.source.getObjectCenterX() + target.getHalfWidth();
		double by0 = collision.source.getObjectCenterY() - target.getHalfHeight();
		double by1 = collision.source.getObjectCenterY() + target.getHalfHeight();

		double targRadius = target.getRadius();
		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;

		// Voronoi Test
		
		// Complete Left
		if (bx1 < cpx)
		{
			// Complete Bottom
			if (by1 < cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx1, by1, cpx, cpy);
			// Complete Top
			else if (by0 > cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx1, by0, cpx, cpy);
			// Straddle Y
			else
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx1, cpy, cpx, cpy);
		}
		// Complete Right
		else if (bx0 > cpx)
		{
			// Complete Bottom
			if (by1 < cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx0, by1, cpx, cpy);
			// Complete Top
			else if (by0 > cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx0, by0, cpx, cpy);
			// Straddle Y
			else
				return testAABBCircleIncidents(incVect, incPoint, targRadius, bx0, cpy, cpx, cpy);
		}
		// Straddle X
		else
		{
			// Complete Bottom
			if (by1 < cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, cpx, by1, cpx, cpy);
			// Complete Top
			else if (by0 > cpy)
				return testAABBCircleIncidents(incVect, incPoint, targRadius, cpx, by0, cpx, cpy);
			// Straddle Y
			else
			{
				double closeX = closerComponent(cpx, bx0, bx1);
				double closeY = closerComponent(cpy, by0, by1);
				if (closeX < closeY)
				{
					double bpx = collision.source.getObjectCenterX();
					if (cpx < bpx)
					{
						incVect.set(-closeX - targRadius, 0);
						incPoint.set(cpx + targRadius, cpy);
					}
					else
					{
						incVect.set(closeX + targRadius, 0);
						incPoint.set(cpx - targRadius, cpy);
					}
				}
				else
				{
					double bpy = collision.source.getObjectCenterY();
					if (cpy < bpy)
					{
						incVect.set(0, -closeY - targRadius);
						incPoint.set(cpx, cpy + targRadius);
					}
					else
					{
						incVect.set(0, closeY + targRadius);
						incPoint.set(cpx, cpy - targRadius);
					}
				}
				return true;
			}

		}
		
	}
	
	/**
	 * Tests if two shapes collide. AABB vs. AABB.
	 * @param source the source shape.
	 * @param target the target shape.
	 * @param collision the collision data object. source and target should already be set.
	 * @return true if the shapes collide, false otherwise.
	 */
	public static boolean testStationaryCollision(AABB source, AABB target, Collision2D<?> collision)
	{
		collision.method = Method.BOX_TO_BOX;

		Vect2D incVect = collision.incidentVector;
		Point2D incPoint = collision.incidentPoint;
		
		double spx = collision.source.getObjectCenterX();
		double tpx = collision.target.getObjectCenterX();
		double shw = source.getHalfWidth();
		double thw = target.getHalfWidth();
		double spy = collision.source.getObjectCenterY();
		double tpy = collision.target.getObjectCenterY();
		double shh = source.getHalfHeight();
		double thh = target.getHalfHeight();
		
		if (spx < tpx) // box to the left.
		{
			if (spx + shw < tpx - thw)
				return false;
			
			if (spy < tpy) // box to the bottom.
			{
				if (spy + shh < tpy - thh)
					return false;
				
				double dx = Math.abs((spx + shw) - (tpx - thw));
				double dy = Math.abs((spy + shh) - (tpy - thh));
				
				if (dx < dy)
				{
					incVect.set(dx, 0);
					double d0 = Math.max(tpy - thh, spy - shh); 
					double d1 = Math.min(tpy + thh, spy + shh); 
					incPoint.set(spx + shw - dx, (d0 + d1) / 2.0);
				}
				else
				{
					incVect.set(0, dy);
					double d0 = Math.max(tpx - thw, spx - shw); 
					double d1 = Math.min(tpx + thw, spx + shw); 
					incPoint.set((d0 + d1) / 2.0, spy + shh - dy);
				}
				return true;
			}
			else // box to the top.
			{
				if (spy - shh > tpy + thh)
					return false;
				
				double dx = Math.abs((spx + shw) - (tpx - thw));
				double dy = Math.abs((tpy + thh) - (spy - shh));
				
				if (dx < dy)
				{
					incVect.set(dx, 0);
					double d0 = Math.max(tpy - thh, spy - shh); 
					double d1 = Math.min(tpy + thh, spy + shh); 
					incPoint.set(spx + shw - dx, (d0 + d1) / 2.0);
				}
				else
				{
					incVect.set(0, -dy);
					double d0 = Math.max(tpx - thw, spx - shw); 
					double d1 = Math.min(tpx + thw, spx + shw); 
					incPoint.set((d0 + d1) / 2.0, spy - shh + dy);
				}
				return true;
			}
		}
		else // box to the right
		{
			if (spx - shw > tpx + thw)
				return false;
	
			if (spy < tpy) // box to the bottom.
			{
				if (spy + shh < tpy - thh)
					return false;
				
				double dx = Math.abs((tpx + thw) - (spx - shw));
				double dy = Math.abs((spy + shh) - (tpy - thh));
				
				if (dx < dy)
				{
					incVect.set(-dx, 0);
					double d0 = Math.max(tpy - thh, spy - shh); 
					double d1 = Math.min(tpy + thh, spy + shh); 
					incPoint.set(spx - shw + dx, (d0 + d1) / 2.0);
				}
				else
				{
					incVect.set(0, dy);
					double d0 = Math.max(tpx - thw, spx - shw); 
					double d1 = Math.min(tpx + thw, spx + shw); 
					incPoint.set((d0 + d1) / 2.0, spy + shh - dy);
				}
				return true;
			}
			else // box to the top.
			{
				if (spy - shh > tpy + thh)
					return false;
				
				double dx = Math.abs((tpx + thw) - (spx - shw));
				double dy = Math.abs((tpy + thh) - (spy - shh));
				
				if (dx < dy)
				{
					incVect.set(-dx, 0);
					double d0 = Math.max(tpy - thh, spy - shh); 
					double d1 = Math.min(tpy + thh, spy + shh); 
					incPoint.set(spx - shw + dx, (d0 + d1) / 2.0);
				}
				else
				{
					incVect.set(0, -dy);
					double d0 = Math.max(tpx - thw, spx - shw); 
					double d1 = Math.min(tpx + thw, spx + shw); 
					incPoint.set((d0 + d1) / 2.0, spy - shh + dy);
				}
				return true;
			}
		}
	}

	/**
	 * Tests collisions using separating axis.
	 * @param bodyA the first body. 
	 * @param bodyB the second body.
	 * @param collision the collision information.
	 * TODO: Incident points for separating axis test.
	 */
	public static boolean testSeparatingAxisCollision(CollisionBody2D bodyA, CollisionBody2D bodyB, Collision2D<?> collision)
	{
		collision.method = Method.SEPARATING_AXIS;

		Cache cache = getCache();
		cache.angleHashReset();
		
		boolean stillGood = true;

		// get separating axes.
		cacheSeparatingAxes(bodyA, bodyB);
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
				stillGood = axisOverlapTest(bodyA, bodyB, axis, cache.vector);
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
	
	/**
	 * Sets incident vectors and points if a collision occurs between
	 * Circles and AABBs.
	 */
	private static boolean testCircleAABBIncidents(Vect2D vect, Point2D point, 
			double srcradius, double spx, double spy, double ax, double ay)
	{
		double dist = distance(spx, spy, ax, ay);
		if (dist < srcradius)
		{
			double dx = ax - spx;
			double dy = ay - spy;
			vect.set(dx, dy);
			vect.setLength(srcradius - dist);
			point.set(ax, ay);
			return true;
		}
		return false;
	}

	/**
	 * Sets incident vectors and points if a collision occurs between
	 * AABBs and Circles.
	 */
	private static boolean testAABBCircleIncidents(Vect2D vect, Point2D point, 
		double targradius, double bx, double by, double cx, double cy)
	{
		double dist = distance(cx, cy, bx, by);
		if (dist < targradius)
		{
			double theta = RMath.getVectorAngleRadians(bx - cx, by - cy);
			vect.set(cx - bx, cy - by);
			vect.setLength(targradius - dist);
			point.set(targradius * Math.cos(theta) + cx, targradius * Math.sin(theta) + cy);
			return true;
		}
		return false;
	}

	/**
	 * Returns the square distance between two points.
	 */
	public static double distance(double pointAX, double pointAY, double pointBX, double pointBY)
	{
		return Math.sqrt(squareDistance(pointAX, pointAY, pointBX, pointBY));
	}

	/**
	 * Returns the square distance between two points.
	 */
	public static double squareDistance(double pointAX, double pointAY, double pointBX, double pointBY)
	{
		return (pointAX - pointBX)*(pointAX - pointBX) + (pointAY - pointBY)*(pointAY - pointBY);
	}

	/**
	 * Returns (val - min) if val is closer to min than max, (max - val) otherwise.
	 * Result is always positive. 
	 */
	public static double closerComponent(double val, double min, double max)
	{
		return Math.abs(val - min) < Math.abs(val - max) ? (val - min) : (max - val);
	}
	

}
