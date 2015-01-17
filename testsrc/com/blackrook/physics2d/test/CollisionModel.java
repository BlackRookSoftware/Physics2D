/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.test;

import com.blackrook.commons.math.Tuple2D;
import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.commons.math.geometry.Vect2D;
import com.blackrook.physics2d.Physics2DModel;
import com.blackrook.physics2d.Shape2D;

public class CollisionModel implements Physics2DModel<CollisionBody>
{
	@Override
	public Iterable<CollisionBody> getAllObjects()
	{
		return null;
	}

	@Override
	public void getObjectCollisionCenter(CollisionBody object, Point2D center)
	{
		center.set(object.x, object.y);
	}

	@Override
	public void getObjectCollisionHalfWidths(CollisionBody object, Tuple2D halfwidths)
	{
		halfwidths.set(object.shape.getHalfWidth(), object.shape.getHalfHeight());
	}

	@Override
	public void getObjectCollisionVelocity(CollisionBody object, Vect2D velocity)
	{
		velocity.set(object.vx, object.vy);
	}

	@Override
	public boolean isObjectInMotion(CollisionBody object)
	{
		return object.vx != 0.0 || object.vy != 0.0;
	}

	@Override
	public double getObjectCollisionSquaredRadius(CollisionBody object)
	{
		return object.shape.getHalfWidth() * object.shape.getHalfWidth() + object.shape.getHalfHeight() * object.shape.getHalfHeight();
	}

	@Override
	public double getObjectCollisionRotationZ(CollisionBody object)
	{
		return object.rotation;
	}

	@Override
	public long getObjectCollisionGroupMask(CollisionBody object)
	{
		return -1L;
	}

	@Override
	public long getObjectCollisionTargetMask(CollisionBody object)
	{
		return -1L;
	}

	@Override
	public Shape2D getObjectCollisionShape(CollisionBody object)
	{
		return object.shape;
	}

	@Override
	public boolean isObjectExcludedFromCollision(CollisionBody object)
	{
		return false;
	}
}
