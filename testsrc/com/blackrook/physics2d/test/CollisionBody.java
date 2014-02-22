/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.test;

import com.blackrook.physics2d.CollisionBody2D;
import com.blackrook.physics2d.Shape2D;

public class CollisionBody implements CollisionBody2D
{
	protected Shape2D shape;
	public float x;
	public float y;
	public float vx;
	public float vy;
	public float rotation;
	
	public CollisionBody(Shape2D shape)
	{
		this.shape = shape;
	}
	
	@Override public int getCollisionGroupMask()		{return -1;}
	@Override public int getCollisionTargetMask()		{return -1;}
	@Override public Shape2D getCollisionShape()		{return shape;}
	@Override public float getCollisionSquaredRadius()	{return (float)shape.getSquareRadius();}
	@Override public float getCollisionVelocityX()		{return vx;}
	@Override public float getCollisionVelocityY()		{return vy;}
	@Override public float getCollisionRotationZ()		{return rotation;}

	@Override public float getObjectRadius() 			{return (float)shape.getRadius();}
	@Override public boolean useObjectRadius() 			{return false;}
	@Override public boolean isExcludedFromCollision()	{return false;}
	@Override public float getObjectCenterX() 			{return x;}
	@Override public float getObjectCenterY() 			{return y;}
	@Override public float getObjectHalfWidth() 		{return shape.getHalfWidth();}
	@Override public float getObjectHalfHeight() 		{return shape.getHalfHeight();}
	@Override public float getObjectHalfDepth() 		{return 0;}
	@Override public float getObjectCenterZ() 			{return 0;}
	@Override public float getObjectSweepX()			{return -vx;}
	@Override public float getObjectSweepY()			{return -vy;}
	@Override public float getObjectSweepZ()			{return 0;}
	@Override public String toString() 					{return shape.toString() + "@("+x+","+y+")";} 

}
