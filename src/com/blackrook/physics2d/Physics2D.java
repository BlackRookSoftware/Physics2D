/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

import com.blackrook.commons.ResettableIterator;
import com.blackrook.commons.list.List;
import com.blackrook.commons.spatialhash.SpatialHash2D;

/**
 * This is the world that each collidable object is added to.
 * @author Matthew Tropiano
 */
public class Physics2D<T extends CollisionBody2D>
{
	/** The spatial hash to use that contains collidable objects. */
	protected SpatialHash2D<CollisionBody2D> collisionHash;
	/** Collision spatial hash iterator. */
	protected ResettableIterator<CollisionBody2D> collisionHashIterator;
	
	/** Collision data. */
	protected Collision2D<T> collision;
	/** List of listeners. */
	protected List<Physics2DListener<T>> systemListeners;

	/** Number of collisions last query. */
	protected int collisionCount;
	/** Are we currently performing a query? */
	protected boolean queryActive;
	
	/** Collision pool. */
	private CollisionBody2D[] intersectingObjects;
	
	/**
	 * Creates a new PhysicsSystem with a blank internal spatial hash and default block size.
	 */
	public Physics2D()
	{
		this(new SpatialHash2D<CollisionBody2D>(128));
	}

	/**
	 * Creates a new PhysicsSystem with a blank internal spatial hash.
	 * @param resolution the resolution of the spatial hash map.
	 */
	public Physics2D(int resolution)
	{
		this(new SpatialHash2D<CollisionBody2D>(resolution));
	}

	/**
	 * Creates a new PhysicsSystem.
	 * @param hash a spatial hash to use for the physics system instead of a new internal spatial hash.
	 */
	public Physics2D(SpatialHash2D<CollisionBody2D> hash)
	{
		collisionHash = hash;
		collision = new Collision2D<T>();
		systemListeners = new List<Physics2DListener<T>>(3);
		intersectingObjects = new CollisionBody2D[200];
	}
	
	/**
	 * Adds a PhysicsSystemListener to this system.
	 * Without one, collisions cannot be handled by the implementing application.
	 * @param listener the listener to add.
	 */
	public void addListener(Physics2DListener<T> listener)
	{
		systemListeners.add(listener);
	}
	
	/**
	 * Removes a PhysicsSystemListener from this system.
	 * @param listener the listener to remove.
	 */
	public void removeListener(Physics2DListener<T> listener)
	{
		systemListeners.remove(listener);
	}
	
	/**
	 * Adds an object to the system.
	 * @param object the object to add.
	 */
	public void addObject(T object)
	{
		collisionHash.addObject(object);
	}

	/**
	 * Removes an object from the system.
	 * @param object the object to remove.
	 */
	public boolean removeObject(T object)
	{
		return collisionHash.removeObject(object);
	}

	/**
	 * Updates this object in the world.
	 * @param object the object to update.
	 */
	public void updateObject(T object)
	{
		collisionHash.updateObject(object);
	}
	
	/**
	 * Updates the system, getting all of the collisions.
	 * Upon each collision, it fires an event with the collision information.
	 */
	public void findCollisions()
	{
		queryActive = true;
		int collisions = 0;
		if (collisionHash != null)
		{
			ResettableIterator<CollisionBody2D> rit = getSceneIterator();
			while (rit.hasNext())
			{
				CollisionBody2D source = rit.next();
				int c = 0;
				if ((c = testCollision(source, collision)) > 0)
					collisions += c;
			}
		}
		collisionCount = collisions;
		queryActive = false;
	}

	/**
	 * Gets a series of collisions with a point.
	 * @param x the point, x-coordinate.
	 * @param y the point, y-coordinate.
	 * @param intersections the output vector of collisions.
	 * @return the number of collisions.
	 */
	public int queryIntersections(double x, double y, CollisionBody2D[] intersections)
	{
		return collisionHash.getIntersections(x, y, intersections, 0);
	}
	
	/**
	 * Gets a series of collisions with an object.
	 * @param body the body to query.
	 * @param intersections the output vector of collisions.
	 * @return the number of collisions.
	 */
	public int queryIntersections(CollisionBody2D body, CollisionBody2D[] intersections)
	{
		return collisionHash.getIntersections(body, intersections, 0);
	}
	
	/**
	 * Returns the number of collisions detected in the last query.
	 */
	public int getCollisionCount()
	{
		return collisionCount;
	}
	
	/**
	 * Returns the number of objects in the physics system.
	 */
	public int getObjectCount()
	{
		return collisionHash.size();
	}

	/**
	 * Returns if this is currently performing a query or not.
	 */
	public boolean isQueryActive()
	{
		return queryActive; 
	}
	
	/**
	 * Tests for one or more collisions with the provided object
	 * and the rest of the objects in the system.  
	 * Upon each collision, it fires an event with the collision information.
	 * @param source the source object.
	 * @param collision the collision data modified by this check.
	 * @return the number of collisions.
	 */
	@SuppressWarnings("unchecked")
	public int testCollision(CollisionBody2D source, Collision2D<T> collision)
	{
		if (source.isExcludedFromCollision())
			return 0;
		
		if (source.getCollisionTargetMask() == 0L)
			return 0;
		
		// TODO: If velocity on objects, find closest possible collision.
		int intersections = collisionHash.getIntersections(source, intersectingObjects, 0);
		int collisions = 0;
		for (int i = 0; i < intersections; i++)
		{
			CollisionBody2D target = intersectingObjects[i];
			
			if (testCollision((T)source, (T)target, collision))
			{
				fireCollision(collision);
				collisions++;
			}
		}
		
		return collisions;
	}
	
	/**
	 * Tests for a collision between two objects.
	 * @param source the source object.
	 * @param target the target object.
	 * @param collision the collision data modified by this check.
	 * @return true if a collision occurred, false otherwise.
	 */
	public boolean testCollision(T source, T target, Collision2D<T> collision)
	{
		if (target.isExcludedFromCollision())
			return false;
		
		if (source.getCollisionTargetMask() == 0 || target.getCollisionGroupMask() == 0)
			return false;
			
		if ((source.getCollisionTargetMask() & target.getCollisionGroupMask()) == 0)
			return false;
		
		collision.source = source;
		collision.target = target;
		long nanos = System.nanoTime();
		boolean out = PhysicsUtils2D.testCollision(source, target, collision);
		collision.calcNanos = System.nanoTime() - nanos;
		return out;
	}
	
	protected ResettableIterator<CollisionBody2D> getSceneIterator()
	{
		if (collisionHashIterator == null)
			collisionHashIterator = collisionHash.iterator();
		else
			collisionHashIterator.reset();
		return collisionHashIterator;
	}
	
	/**
	 * Fires a collision event.
	 */
	protected void fireCollision(Collision2D<T> collision)
	{
		for (int i = 0; i < systemListeners.size(); i++)
			systemListeners.getByIndex(i).handleCollision(collision);
	}
	
}
