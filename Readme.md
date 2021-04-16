First Name: Chenhao

Last Name: Li

Solution to Question 4:

Recall the definition of the Frobenius norm
$$
||A||_F = \sqrt{\operatorname{tr}(A^\top A)}
$$
Hence, by linearality of trace, we get
$$
\operatorname{tr}(\mathcal{F}^\top \mathcal{F} - I) = \operatorname{tr}(\mathcal{F}^\top \mathcal{F}) - \operatorname{tr}(I) = \operatorname{tr}(\mathcal{F}^\top \mathcal{F}) - 2 = ||\mathcal{F}||_F^2 - 2
$$


Solution to Question 10:

The objective function has yet only addressed the cost induced by position deviation. As a result, the gradient applied during gradient descend method on the input components corresponding to positions ($x_L, y_L, x_R, y_R$) are significantly larger than those corresponding to the angles ($\theta_L, \theta_R$). And this results in the fact that the handles translate dramatically while barely rotate. In other words, the large difference between the gradient scales has caused an unbalanced optimization direction, where translation is emphasized while rotation is almost neglected.


Solution to Question 11:



---

Assignment writeup: http://crl.ethz.ch/teaching/computational-motion-21/slides/tutorial-a4.pdf

---

Could use ./build.sh on Linux/MacOS
