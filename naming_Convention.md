# Naming Convention

1. Files:
   - Use lowercase letters for file names.
   - Separate words with underscores ("_") to enhance readability.
   - Choose descriptive file names that reflect the content and purpose of the file.
   - Use appropriate file extensions based on the file type (e.g., `.cpp` for C++ source files, `.py` for Python scripts, `.launch` for launch files).

2. Modules and Packages:
   - Use lowercase letters for module and package names.
   - Separate words with underscores ("_") to enhance readability.
   - Choose descriptive names that convey the purpose and functionality of the module or package.
   - Consider using a hierarchical naming structure for packages to indicate their relationship (e.g., `my_package.subpackage`).

3. Variables:
   - Use lowercase letters for variable names.
   - Separate words with underscores ("_") to enhance readability.
   - Choose descriptive names that clearly indicate the purpose and content of the variable.
   - Use meaningful nouns or noun phrases for variable names, avoiding abbreviations or excessively short names.

4. Functions and Methods:
   - Use lowercase letters for function and method names.
   - Separate words with underscores ("_") to enhance readability.
   - Choose descriptive names that indicate the action performed by the function or method.
   - Use verbs or verb phrases to describe the functionality of the function or method.

5. Classes:
   - Use CamelCase for class names.
   - Begin the class name with an uppercase letter.
   - Use descriptive and meaningful names that represent the purpose and responsibilities of the class.
   - Avoid generic names and aim for specificity.

6. Test Cases:
   - Prefix test case names with "test_" to distinguish them from other functions or methods.
   - Follow the same naming conventions for variables, functions, and classes within the test case.
   - Use descriptive names that reflect the functionality being tested and the expected outcome.
   - Use underscores ("_") to separate words for improved readability.

7. Constants:
   - Use uppercase letters for constant names.
   - Separate words with underscores ("_") to enhance readability.
   - Use descriptive names that clearly convey the purpose and content of the constant.
   - Consider placing constants in a separate file or module for better organization.

8. Acronyms and Abbreviations:
   - When using acronyms or abbreviations, follow consistent capitalization rules.
   - Use uppercase letters for acronyms with two or fewer letters (e.g., `IO`, `GUI`).
   - Use uppercase for the first letter only in acronyms with three or more letters (e.g., `HTTP`, `XML`).

9. ROS-Specific Conventions:
   - Follow the ROS naming conventions for topics, services, and actions, which typically use lowercase letters and underscores (e.g., `/sensor_data`, `/robot_control`).
   - Adhere to the ROS naming conventions for message and service types, which generally use CamelCase with an initial uppercase letter (e.g., `SensorData`, `RobotControl`).

Remember that these naming guidelines serve as a foundation for consistency and clarity within your ROS projects. Adapt them to your team's specific needs and requirements while ensuring that the chosen names are meaningful and descriptive. Consistency in naming helps improve code readability, maintainability, and collaboration among team members.
