/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d.test;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.event.MouseInputListener;

import com.blackrook.commons.math.RMath;
import com.blackrook.commons.math.geometry.Line2D;
import com.blackrook.commons.math.geometry.Point2D;
import com.blackrook.commons.math.geometry.Vect2D;
import com.blackrook.physics2d.Physics2DUtils;
import com.blackrook.physics2d.Shape2D;
import com.blackrook.physics2d.shape2d.AABB2D;
import com.blackrook.physics2d.shape2d.Circle;
import com.blackrook.physics2d.shape2d.Polygon;

public class ProjectionTest {

	static final CollisionModel model = new CollisionModel(); 

	public static void main(String[] args)
	{
		
		Shape2D[] shapes = {
				new Circle(30),
				new AABB2D(25, 25),
				new AABB2D(12, 12),
				new Polygon(new Point2D[]{
					new Point2D(-40,  15),
					new Point2D(0,    40),
					new Point2D(40,   15),
					new Point2D(20,  -40),
					new Point2D(-20, -40),
			}),
				new Polygon(new Point2D[]{
					new Point2D(-20, 20),
					new Point2D(20, 20),
					new Point2D(20, -20),
					new Point2D(-20, -20)
			})
		};

		CollisionBody cb1 = new CollisionBody(shapes[3]);
		cb1.rotation = 45f;
		CollisionBody cb2 = new CollisionBody(shapes[2]); 
		makeTestWindow(cb1, cb2, "Test");
	}

	private static JFrame makeTestWindow(CollisionBody body, CollisionBody body2, String name)
	{
		JFrame out = new JFrame(name);
		out.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		out.add(new TestCanvas(body, body2));
		out.setResizable(true);
		out.pack();
		out.setVisible(true);
		return out;
	}

	private static class TestCanvas extends Canvas implements MouseInputListener
	{
		private static final long serialVersionUID = -7369670472224047283L;

		CollisionBody body;
		CollisionBody body2;
		Line2D proj;
		Line2D proj2;
		Vect2D normal;
		Point2D tp1;
		Point2D tp2;
		Vect2D tv;
		BufferedImage bi;

		
		TestCanvas(CollisionBody body, CollisionBody body2)
		{
			this.body = body;
			this.body2 = body2;
			proj = new Line2D();
			proj2 = new Line2D();
			normal = new Vect2D();
			tp1 = new Point2D();
			tp2 = new Point2D();
			tv = new Vect2D();
			setPreferredSize(new Dimension(640,480));
			addMouseMotionListener(this);
			addMouseListener(this);
		}
		
		public void update(Graphics g)
		{
			Graphics2D g2d = (Graphics2D)g;
			if (bi == null || bi.getWidth() != getWidth() || bi.getHeight() != getHeight())
				bi = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_ARGB);
			drawToImage(bi);
			g2d.drawImage(bi, null, 0, 0);
		}
		
		public void drawToImage(BufferedImage bi)
		{
			Graphics2D g2d = (Graphics2D)bi.getGraphics();
			g2d.setColor(Color.BLACK);
			g2d.scale(1, -1);
			g2d.translate(0, -getHeight());
			g2d.fillRect(0, 0, getWidth(), getHeight());
			
			double rotationZ = model.getObjectCollisionRotationZ(body);
			model.getObjectCollisionCenter(body, tp1);
			model.getObjectCollisionVelocity(body, tv);
			g2d.setColor(Color.GREEN);
			drawShape(g2d, model.getObjectCollisionShape(body), tp1.x, tp1.y, tv.x, tv.y, rotationZ);

			g2d.setColor(Color.WHITE);
			drawProjection(g2d, proj.pointA, proj.pointB);

			rotationZ = model.getObjectCollisionRotationZ(body2);
			model.getObjectCollisionCenter(body2, tp1);
			model.getObjectCollisionVelocity(body2, tv);
			g2d.setColor(Color.GREEN);
			drawShape(g2d, model.getObjectCollisionShape(body2), tp1.x, tp1.y, tv.x, tv.y, rotationZ);
			
			g2d.setColor(Color.WHITE);
			drawProjection(g2d, proj2.pointA, proj2.pointB);
			g2d.dispose();
		}

		
		public void drawProjection(Graphics2D g2d, Point2D pt0, Point2D pt1)
		{
			double x = getWidth()/2;
			double y = getHeight()/2;
			int ix0 = (int)(x+pt0.x);
			int ix1 = (int)(x+pt1.x);
			int iy0 = (int)(y+pt0.y);
			int iy1 = (int)(y+pt1.y);
			g2d.drawLine(ix0, iy0, ix1, iy1);
			g2d.fillOval(ix0-2, iy0-2, 4, 4);
			g2d.fillOval(ix1-2, iy1-2, 4, 4);
		}
		
		public void drawShape(Graphics2D g2d, Shape2D s, double x, double y, double vx, double vy, double rotation)
		{
			int ix = (int)x + getWidth()/2;
			int iy = (int)y + getHeight()/2;
			int ir = (int)s.getRadius();
			int ihw = (int)s.getHalfWidth();
			int ihh = (int)s.getHalfHeight();
			int ivx = (int)vx;
			int ivy = (int)vy;
			if (s instanceof Circle)
			{
				g2d.drawOval(ix-ir, iy-ir, ir*2, ir*2);
				if (vx != 0.0 || vy != 0.0)
					g2d.drawOval(ix-ir-ivx, iy-ir-ivy, ir*2, ir*2);
			}
			else if (s instanceof AABB2D)
			{
				g2d.drawRect(ix-ihw, iy-ihh, ihw*2, ihh*2);
				if (vx != 0.0 || vy != 0.0)
					g2d.drawRect(ix-ihw-ivx, iy-ihh-ivy, ihw*2, ihh*2);
			}
			else if (s instanceof Polygon)
			{
				Polygon p = (Polygon)s;
				Point2D[] pts = p.getPoints();
				for (int n = 0; n < pts.length; n++)
				{
					tp1.set(pts[n]);
					tp1.rotateZ((float)RMath.degToRad(rotation));
					tp2.set(pts[(n+1)%pts.length]);
					tp2.rotateZ((float)RMath.degToRad(rotation));
					g2d.drawLine((int)tp1.x+ix, (int)tp1.y+iy, (int)tp2.x+ix,(int)tp2.y+iy);
				}
				if (vx != 0.0 || vy != 0.0)
				{
					for (int n = 0; n < pts.length; n++)
					{
						tp1.set(pts[n]);
						tp1.rotateZ((float)RMath.degToRad(rotation));
						tp2.set(pts[(n+1)%pts.length]);
						tp2.rotateZ((float)RMath.degToRad(rotation));
						g2d.drawLine((int)tp1.x+ix-ivx, (int)tp1.y+iy-ivy, (int)tp2.x+ix-ivx,(int)tp2.y+iy-ivy);
					}
				}
			}
		}

		@Override
		public void mouseMoved(MouseEvent e) {
			normal.x = e.getX()-(getWidth()/2);
			normal.y = -(e.getY()-(getHeight()/2));
			normal.normalize();
			long time = 0;
			
			time = System.nanoTime();
			
			Shape2D shape = model.getObjectCollisionShape(body);
			
			if (shape instanceof Circle)
				Physics2DUtils.projectCircle(model, (Circle)shape, body, normal, proj);
			else if (shape instanceof AABB2D)
				Physics2DUtils.projectAABB(model, (AABB2D)shape, body, normal, proj);
			else if (shape instanceof Polygon)
				Physics2DUtils.projectPolygon(model, (Polygon)shape, body, normal, proj);
			time = System.nanoTime() - time;
			System.out.print(time + "ns ");
			
			shape = model.getObjectCollisionShape(body2);

			time = System.nanoTime();
			if (shape instanceof Circle)
				Physics2DUtils.projectCircle(model, (Circle)shape, body2, normal, proj2);
			else if (shape instanceof AABB2D)
				Physics2DUtils.projectAABB(model, (AABB2D)shape, body2, normal, proj2);
			else if (shape instanceof Polygon)
				Physics2DUtils.projectPolygon(model, (Polygon)shape, body2, normal, proj2);
			time = System.nanoTime() - time;
			System.out.println(time + "ns");
			
			repaint();
	}

		@Override
		public void mouseDragged(MouseEvent e) {
			body.x = e.getX()-(getWidth()/2);
			body.y = -(e.getY()-(getHeight()/2));
			repaint();
	}

		@Override
		public void mouseClicked(MouseEvent e){
	}

		@Override
		public void mouseEntered(MouseEvent e) {
	}

		@Override
		public void mouseExited(MouseEvent e) {
	}

		@Override
		public void mousePressed(MouseEvent e) {
	}

		@Override
		public void mouseReleased(MouseEvent e) {
	}
		
	}

}
