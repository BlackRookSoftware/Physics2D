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
	public double x;
	public double y;
	public double vx;
	public double vy;
	public double rotation;
	
	public CollisionBody(Shape2D shape)
	{
		this.shape = shape;
	}
	
	@Override public long getCollisionGroupMask()		{return -1;}
	@Override public long getCollisionTargetMask()		{return -1;}
	@Override public Shape2D getCollisionShape()		{return shape;}
	@Override public double getCollisionSquaredRadius()	{return shape.getSquareRadius();}
	@Override public double getCollisionVelocityX()		{return vx;}
	@Override public double getCollisionVelocityY()		{return vy;}
	@Override public double getCollisionRotationZ()		{return rotation;}

	@Override public double getObjectRadius() 			{return shape.getRadius();}
	@Override public boolean useObjectRadius() 			{return false;}
	@Override public boolean isExcludedFromCollision()	{return false;}
	@Override public double getObjectCenterX() 			{return x;}
	@Override public double getObjectCenterY() 			{return y;}
	@Override public double getObjectHalfWidth() 		{return shape.getHalfWidth();}
	@Override public double getObjectHalfHeight() 		{return shape.getHalfHeight();}
	@Override public double getObjectHalfDepth() 		{return 0;}
	@Override public double getObjectCenterZ() 			{return 0;}
	@Override public double getObjectSweepX()			{return -vx;}
	@Override public double getObjectSweepY()			{return -vy;}
	@Override public double getObjectSweepZ()			{return 0;}
	@Override public String toString() 					{return shape.toString() + "@("+x+","+y+")";} 

}
